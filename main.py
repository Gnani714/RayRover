# main.py
import pygame, numpy as np
from control.motion import ReactiveController
from control.path_follow import PurePursuitFollower
from planner.a_star import grid_from_obstacles, a_star, grid_to_world
from world.obstacles import get_obstacles, draw_obstacles
from sensors.lidar import Lidar
from viz.lidar_viz import LidarScanner

pygame.init()
W, H = 800, 600
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("RayRover — Reactive vs Planner")

# --- LIDAR ---
lidar = Lidar(lidar_range=150, num_rays=64, step=2)
scanner = LidarScanner(lidar, show_sweep=True)

# --- Robot & controllers ---
robot = {"x": 100.0, "y": 100.0, "theta": 0.0}
reactive = ReactiveController(robot)                            # has .speed
follower = PurePursuitFollower(robot, lookahead=28.0, speed=2.2, world=(W, H))  # has .speed

# --- World ---
obstacles = get_obstacles()

clock = pygame.time.Clock()
running = True
path_trail = []
mode = "reactive"   # 'reactive' or 'planner'
goal = None
CELL = 16           # grid cell size (px)

# --- Speed control state ---
SPEED_PRESETS = [("SLOW", 1.2), ("NORMAL", 2.2), ("FAST", 3.2)]
current_speed_idx = 1   # start at NORMAL

def apply_speed(speed_value: float):
    """Apply speed to both controllers."""
    reactive.speed = float(speed_value)
    follower.speed = float(speed_value)

apply_speed(SPEED_PRESETS[current_speed_idx][1])

# --- UI: speed buttons (top-right) ---
BTN_W, BTN_H, BTN_PAD = 90, 28, 10
btn_rects = []
def layout_buttons():
    x = W - (BTN_W + BTN_PAD)
    y = BTN_PAD
    rects = []
    for _ in SPEED_PRESETS[::-1]:  # draw FAST at top by reversing, looks nice
        rects.append(pygame.Rect(x, y, BTN_W, BTN_H))
        y += BTN_H + 6
    return list(reversed(rects))   # restore order to match SPEED_PRESETS
btn_rects = layout_buttons()

def draw_speed_buttons(mouse_pos):
    font = pygame.font.SysFont(None, 18)
    for i, (label, _) in enumerate(SPEED_PRESETS):
        r = btn_rects[i]
        is_selected = (i == current_speed_idx)
        is_hover = r.collidepoint(mouse_pos)
        # colors
        bg = (220, 235, 255) if is_selected else (235, 235, 235)
        bd = (40, 120, 255) if is_selected else (180, 180, 180)
        if is_hover and not is_selected:
            bg = (245, 245, 245)
            bd = (120, 120, 120)

        pygame.draw.rect(screen, bg, r, border_radius=8)
        pygame.draw.rect(screen, bd, r, 2, border_radius=8)
        txt = font.render(label, True, (30, 30, 30))
        screen.blit(txt, (r.x + (r.w - txt.get_width())//2, r.y + (r.h - txt.get_height())//2))

def set_speed_by_index(idx):
    global current_speed_idx
    current_speed_idx = max(0, min(len(SPEED_PRESETS)-1, idx))
    apply_speed(SPEED_PRESETS[current_speed_idx][1])

def nudge_speed(delta):
    # fine control via +/- keys (clamped 0.5..5.0)
    new_speed = max(0.5, min(5.0, follower.speed + delta))
    apply_speed(new_speed)

def draw_path(screen, pts, color=(50, 120, 50)):
    if len(pts) > 1:
        pygame.draw.lines(screen, color, False, [(int(x), int(y)) for x, y in pts], 3)

while running:
    mouse_pos = pygame.mouse.get_pos()
    screen.fill((245, 245, 245))

    # --- Events ---
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        elif e.type == pygame.KEYDOWN:
            if e.key == pygame.K_r: mode = "reactive"
            if e.key == pygame.K_p: mode = "planner"
            # NEW SPEED CONTROL
            if e.key == pygame.K_UP:          # ↑ arrow
                nudge_speed(+0.2)
            if e.key == pygame.K_DOWN:        # ↓ arrow
                nudge_speed(-0.2)

            # Preset selector
            if e.key == pygame.K_1: set_speed_by_index(0)
            if e.key == pygame.K_2: set_speed_by_index(1)
            if e.key == pygame.K_3: set_speed_by_index(2)
            
        elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
            # click on buttons first
            clicked_button = None
            for i, r in enumerate(btn_rects):
                if r.collidepoint(e.pos):
                    clicked_button = i
                    break
            if clicked_button is not None:
                set_speed_by_index(clicked_button)
            else:
                # Set navigation goal
                goal = e.pos
                grid, cols, rows = grid_from_obstacles(W, H, CELL, obstacles)
                sx, sy = int(robot["x"] // CELL), int(robot["y"] // CELL)
                gx, gy = int(goal[0] // CELL), int(goal[1] // CELL)
                path_cells = a_star(grid, (sx, sy), (gx, gy))
                if path_cells:
                    waypoints = grid_to_world(path_cells, CELL)
                    waypoints = waypoints[::2] + [waypoints[-1]]
                    follower.set_path(waypoints)
                    mode = "planner"  # auto-switch

    # --- Draw world ---
    draw_obstacles(screen, obstacles)

    # --- Update robot pose ---
    if mode == "reactive":
        robot = reactive.update(obstacles, screen=None)
    else:
        robot = follower.update()

    # --- LIDAR scan + visualization ---
    distances, hits, angles = lidar.scan(robot, obstacles)
    scanner.draw(screen, robot, distances, hits, angles)

    # --- Path trail ---
    path_trail.append((int(robot["x"]), int(robot["y"])))
    if len(path_trail) > 80:
        path_trail.pop(0)
    for p in path_trail:
        pygame.draw.circle(screen, (255, 150, 0), p, 2)

    # --- Planned path & lookahead point ---
    if follower.path:
        draw_path(screen, follower.path, color=(0, 140, 0))
        idx = min(follower.target_idx, len(follower.path) - 1)
        lx, ly = follower.path[idx]
        pygame.draw.circle(screen, (0, 0, 0), (int(lx), int(ly)), 4)

    # --- Goal marker ---
    if goal:
        pygame.draw.circle(screen, (220, 0, 0), (int(goal[0]), int(goal[1])), 6, 2)

    # --- Robot body + heading ---
    pygame.draw.circle(screen, (0, 120, 255), (int(robot["x"]), int(robot["y"])), 10)
    pygame.draw.line(
        screen, (0, 0, 0),
        (robot["x"], robot["y"]),
        (robot["x"] + 15 * np.cos(robot["theta"]), robot["y"] + 15 * np.sin(robot["theta"])),
        2
    )

    # --- HUD ---
    font = pygame.font.SysFont(None, 20)
    speed_lbl = f"{SPEED_PRESETS[current_speed_idx][0]} ({follower.speed:.1f} px/s)"
    hud = font.render(
        f"Mode: {mode.upper()}  |  Speed: {speed_lbl}  |  L-Click: Set Goal  |  P: Planner  R: Reactive  |  +/- fine adjust  |  1/2/3 presets",
        True, (30, 30, 30)
    )
    screen.blit(hud, (10, H - 26))

    # --- Speed buttons (top-right) ---
    draw_speed_buttons(mouse_pos)

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
