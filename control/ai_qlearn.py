# control/ai_qlearn.py
import numpy as np
from collections import defaultdict

def _bin(v, edges):
    for i,e in enumerate(edges):
        if v < e: return i
    return len(edges)

class QLearnController:
    """
    Q-learning with strong goal shaping + safety override + stall recovery.
    State: min LIDAR in (left/front/right) + bearing bin
    Actions: left / straight / right (always forward)
    """
    def __init__(self, robot, lidar, world=(800,600),
                 alpha=0.35, gamma=0.96,
                 eps_start=0.40, eps_min=0.02, eps_decay=0.996,
                 turn=np.radians(20), speed=2.8, goal_radius=20):
        self.robot = robot
        self.lidar = lidar
        self.W, self.H = world
        self.alpha, self.gamma = alpha, gamma
        self.eps, self.eps_min, self.eps_decay = eps_start, eps_min, eps_decay
        self.turn, self.speed = float(turn), float(speed)
        self.goal_radius = goal_radius
        self.Q = defaultdict(lambda: np.zeros(3, dtype=float))
        self.prev_goal_dist = None
        self.no_prog = 0         # stall counter

        # distance & bearing bins
        self.dist_edges = [20, 35, 55, 85, 120, 170, 230]
        self.bearing_edges = list(np.linspace(-np.pi, np.pi, 13))[1:-1]

    def set_speed(self, v): self.speed = float(v)

    # ---------------- helpers ----------------
    def _sector_min(self, readings, angles, a0, a1):
        ang = (angles + np.pi) % (2*np.pi) - np.pi
        a0 = ((a0 + np.pi) % (2*np.pi)) - np.pi
        a1 = ((a1 + np.pi) % (2*np.pi)) - np.pi
        mask = (ang >= a0) & (ang <= a1) if a0 <= a1 else ((ang >= a0) | (ang <= a1))
        vals = readings[mask]
        return float(np.min(vals)) if np.any(mask) else float(np.min(readings))

    def _collision(self, x, y, obstacles, radius=9):
        for r in obstacles:
            if r.collidepoint(x, y): return True
            if (x > r.left - radius and x < r.right + radius and
                y > r.top  - radius and y < r.bottom + radius):
                cx = min(max(x, r.left), r.right)
                cy = min(max(y, r.top),  r.bottom)
                if (x-cx)**2 + (y-cy)**2 <= radius**2: return True
        m=6
        return (x<=m or x>=self.W-m or y<=m or y>=self.H-m)

    def _state(self, robot, obstacles, goal):
        readings, _, angles = self.lidar.scan(robot, obstacles)
        front = self._sector_min(readings, angles, np.radians(-20),  np.radians(20))
        left  = self._sector_min(readings, angles, np.radians(60),   np.radians(120))
        right = self._sector_min(readings, angles, np.radians(-120), np.radians(-60))
        fl, ff, fr = _bin(left, self.dist_edges), _bin(front, self.dist_edges), _bin(right, self.dist_edges)

        if goal is None:
            bbin = len(self.bearing_edges)//2; bearing = 0.0; gdist=None
        else:
            gx, gy = goal; x,y,th = robot["x"], robot["y"], robot["theta"]
            ang_to_goal = np.arctan2(gy - y, gx - x)
            bearing = (ang_to_goal - th + np.pi) % (2*np.pi) - np.pi
            bbin = int(np.searchsorted(self.bearing_edges, bearing, side='right'))
            gdist = np.hypot(gx - x, gy - y)
        return (fl, ff, fr, bbin), bearing, gdist

    # --------------- RL core -----------------
    def _choose(self, s):
        if np.random.rand() < self.eps: return np.random.randint(0,3)
        return int(np.argmax(self.Q[s]))

    def _apply(self, a, x, y, th):
        if a==0: th += self.turn
        elif a==2: th -= self.turn
        x += self.speed * np.cos(th); y += self.speed * np.sin(th)
        return x, y, th

    def update(self, obstacles, goal=None):
        s, bearing, gdist = self._state(self.robot, obstacles, goal)
        a = self._choose(s)

        # ---- safety override (strong) ----
        readings, _, angles = self.lidar.scan(self.robot, obstacles)
        front = self._sector_min(readings, angles, np.radians(-18), np.radians(18))
        left  = self._sector_min(readings, angles, np.radians(60),  np.radians(120))
        right = self._sector_min(readings, angles, np.radians(-120), np.radians(-60))
        if front < 30: a = 0 if left > right else 2
        if front < 16 and a==1: a = 0 if left > right else 2

        # ---- transition ----
        x,y,th = self.robot["x"], self.robot["y"], self.robot["theta"]
        nx,ny,nth = self._apply(a, x,y,th)

        # ---- reward ----
        r = -0.01; done = False
        if goal is not None:
            g_now = np.hypot(goal[0]-nx, goal[1]-ny)
            if g_now < self.goal_radius: r += 1.0; done=True
            if self.prev_goal_dist is not None:
                prog = float(self.prev_goal_dist - g_now)
                r += 0.15 * np.clip(prog/10.0, -1.0, 1.0)  # stronger progress pull
                self.no_prog = 0 if prog > 1.0 else self.no_prog + 1
            else:
                self.no_prog = 0
            r += 0.05 * np.cos(bearing)  # face the goal = bonus
            self.prev_goal_dist = g_now
        else:
            self.prev_goal_dist = None; self.no_prog = 0

        if self._collision(nx,ny,obstacles):
            r -= 1.0; done = True; nth += np.pi/2 * (1 if left>right else -1)

        # ---- stall recovery ----
        if not done and self.no_prog >= 40:   # ~1.3s at 30 FPS
            nth += np.pi/2 * (1 if left>right else -1)
            self.no_prog = 0

        # ---- Q update ----
        next_robot = {"x": nx, "y": ny, "theta": nth}
        s2, _, _ = self._state(next_robot, obstacles, goal)
        best_next = np.max(self.Q[s2])
        td = r + self.gamma*best_next - self.Q[s][a]
        self.Q[s][a] += self.alpha * td

        # ---- commit & epsilon ----
        if not done: self.robot.update({"x": nx, "y": ny, "theta": nth})
        else:        self.robot["theta"] = nth
        self.eps = max(self.eps_min, self.eps * self.eps_decay)
        return self.robot
