# main.py
import pygame, numpy as np
from control.motion import ReactiveController
from control.path_follow import PurePursuitFollower
from control.ai_qlearn import QLearnController
from planner.a_star import grid_from_obstacles, a_star, grid_to_world
from world.obstacles import get_obstacles, draw_obstacles
from sensors.lidar import Lidar
from viz.lidar_viz import LidarScanner

pygame.init()
W, H = 800, 600
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("RayRover — Reactive / Planner / AI")

# --- LIDAR ---
lidar = Lidar(lidar_range=150, num_rays=64, step=2)
scanner = LidarScanner(lidar, show_sweep=True)

# --- Robot & controllers ---
robot = {"x": 100.0, "y": 100.0, "theta": 0.0}
reactive = ReactiveController(robot)
follower = PurePursuitFollower(robot, lookahead=28.0, speed=2.2, world=(W, H))
ai_ctrl  = QLearnController(robot, lidar, world=(W, H), speed=2.2)

# --- World ---
obstacles = get_obstacles()

clock = pygame.time.Clock()
running = True
path_trail = []
mode = "reactive"   # 'reactive' | 'planner' | 'ai'
goal = None
CELL = 16

# --- Speed presets / buttons (unchanged from your version) ---
SPEED_PRESETS = [("SLOW", 1.2), ("NORMAL", 2.2), ("FAST", 3.2)]
current_speed_idx = 1

def apply_speed(v):
    reactive.speed = float(v)
    follower.speed = float(v)
    ai_ctrl.set_speed(float(v))
apply_speed(SPEED_PRESETS[current_speed_idx][1])

BTN_W, BTN_H, BTN_PAD = 90, 28, 10
def layout_buttons():
    x = W - (BTN_W + BTN_PAD); y = BTN_PAD; rects=[]
    for _ in SPEED_PRESETS[::-1]:
        rects.append(pygame.Rect(x, y, BTN_W, BTN_H)); y += BTN_H + 6
    return list(reversed(rects))
btn_rects = layout_buttons()

def draw_speed_buttons(mouse_pos):
    font = pygame.font.SysFont(None, 18)
    for i,(lbl,_) in enumerate(SPEED_PRESETS):
        r = btn_rects[i]
        sel = (i==current_speed_idx); hov=r.collidepoint(mouse_pos)
        bg = (220,235,255) if sel else (235,235,235)
        bd = (40,120,255) if sel else (180,180,180)
        if hov and not sel: bg=(245,245,245); bd=(120,120,120)
        pygame.draw.rect(screen, bg, r, border_radius=8)
        pygame.draw.rect(screen, bd, r, 2, border_radius=8)
        txt = font.render(lbl, True, (30,30,30))
        screen.blit(txt, (r.x+(r.w-txt.get_width())//2, r.y+(r.h-txt.get_height())//2))

def set_speed_by_index(i):
    global current_speed_idx
    current_speed_idx = max(0, min(len(SPEED_PRESETS)-1, i))
    apply_speed(SPEED_PRESETS[current_speed_idx][1])

def nudge_speed(dv):
    newv = max(0.5, min(5.0, follower.speed + dv))
    apply_speed(newv)

def draw_path(screen, pts, color=(50,120,50)):
    if len(pts)>1:
        pygame.draw.lines(screen, color, False, [(int(x),int(y)) for x,y in pts], 3)

while running:
    mouse_pos = pygame.mouse.get_pos()
    screen.fill((245,245,245))

    # --- events ---
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        elif e.type == pygame.KEYDOWN:
            if e.key == pygame.K_r: mode = "reactive"
            if e.key == pygame.K_p: mode = "planner"
            if e.key == pygame.K_q: mode = "ai"       # NEW: AI mode
            if e.key == pygame.K_UP:   nudge_speed(+0.2)
            if e.key == pygame.K_DOWN: nudge_speed(-0.2)
            if e.key == pygame.K_1: set_speed_by_index(0)
            if e.key == pygame.K_2: set_speed_by_index(1)
            if e.key == pygame.K_3: set_speed_by_index(2)
        elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            # speed buttons?
            clicked = None
            for i,r in enumerate(btn_rects):
                if r.collidepoint(e.pos): clicked=i; break
            if clicked is not None:
                set_speed_by_index(clicked)
            else:
                # set goal for planner and AI shaping
                goal = e.pos
                # planner path
                grid, _, _ = grid_from_obstacles(W, H, CELL, obstacles)
                sx, sy = int(robot["x"]//CELL), int(robot["y"]//CELL)
                gx, gy = int(goal[0]//CELL), int(goal[1]//CELL)
                cells = a_star(grid, (sx,sy), (gx,gy))
                if cells:
                    waypoints = grid_to_world(cells, CELL)
                    waypoints = waypoints[::2] + [waypoints[-1]]
                    follower.set_path(waypoints)

    # world
    draw_obstacles(screen, obstacles)

    hint = None

    # update pose
    if mode == "reactive":
        robot = reactive.update(obstacles, screen=None)
    elif mode == "planner":
        robot = follower.update()
    else:  # AI
        if follower.path:
            idx = min(follower.target_idx + 3, len(follower.path)-1)  # a few steps ahead
            hint = follower.path[idx]
        robot = ai_ctrl.update(obstacles, hint or goal)

    # lidar viz
    distances, hits, angles = lidar.scan(robot, obstacles)
    scanner.draw(screen, robot, distances, hits, angles)

    # trail
    path_trail.append((int(robot["x"]), int(robot["y"])))
    if len(path_trail)>80: path_trail.pop(0)
    for p in path_trail: pygame.draw.circle(screen, (255,150,0), p, 2)

    # planned path viz
    if follower.path:
        draw_path(screen, follower.path, color=(0,140,0))
        idx = min(follower.target_idx, len(follower.path)-1)
        lx, ly = follower.path[idx]
        pygame.draw.circle(screen, (0,0,0), (int(lx), int(ly)), 4)

    # goal
    if goal:
        pygame.draw.circle(screen, (220,0,0), (int(goal[0]), int(goal[1])), 6, 2)

    # robot
    pygame.draw.circle(screen, (0,120,255), (int(robot["x"]), int(robot["y"])), 10)
    pygame.draw.line(screen, (0,0,0),
        (robot["x"], robot["y"]),
        (robot["x"] + 15*np.cos(robot["theta"]), robot["y"] + 15*np.sin(robot["theta"])), 2)

    # HUD
    font = pygame.font.SysFont(None, 20)
    speed_lbl = f"{SPEED_PRESETS[current_speed_idx][0]} ({follower.speed:.1f})"
    hud = font.render(
        f"Mode: {mode.upper()} | Speed: {speed_lbl} | L-Click: Goal | P=Planner R=Reactive Q=AI | AI ε={ai_ctrl.eps:.2f}",
        True, (30,30,30)
    )
    screen.blit(hud, (10, H-26))

    draw_speed_buttons(mouse_pos)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
