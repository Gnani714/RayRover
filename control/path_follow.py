import numpy as np

class PurePursuitFollower:
    def __init__(self, robot, lookahead=25.0, speed=2.0, world=(800, 600)):
        self.robot = robot
        self.lookahead = float(lookahead)
        self.speed = float(speed)
        self.W, self.H = world
        self.path = []
        self.target_idx = 0

    def set_path(self, waypoints):
        self.path = list(waypoints) if waypoints else []
        self.target_idx = 0

    def update(self):
        if not self.path:
            return self.robot

        x, y, th = self.robot["x"], self.robot["y"], self.robot["theta"]

        # Find lookahead target
        while self.target_idx < len(self.path) - 1:
            tx, ty = self.path[self.target_idx]
            if np.hypot(tx - x, ty - y) >= self.lookahead:
                break
            self.target_idx += 1

        tx, ty = self.path[min(self.target_idx, len(self.path) - 1)]
        angle_to_target = np.arctan2(ty - y, tx - x)
        err = (angle_to_target - th + np.pi) % (2 * np.pi) - np.pi

        # steering + forward motion
        th += 0.15 * err
        x += self.speed * np.cos(th)
        y += self.speed * np.sin(th)

        # Reached goal?
        if self.target_idx >= len(self.path) - 1 and np.hypot(tx - x, ty - y) < 12:
            self.path = []
            self.target_idx = 0

        self.robot.update({"x": x, "y": y, "theta": th})
        return self.robot
