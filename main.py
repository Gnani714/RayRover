import pygame
import numpy as np
from control.motion import MotionController
from world.obstacles import get_obstacles, draw_obstacles

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Autonomous Robot with LIDAR Visualization")

# Robot initialization
robot = {"x": 100, "y": 100, "theta": 0}
controller = MotionController(robot)
obstacles = get_obstacles()

clock = pygame.time.Clock()
running = True
path = []

# LIDAR settings
lidar_range = 120   # pixels
lidar_beams = 36    # number of rays around the robot

def draw_lidar(screen, robot, obstacles):
    """Draw LIDAR sensor beams around the robot."""
    for angle in np.linspace(0, 2 * np.pi, lidar_beams):
        end_x = robot["x"] + lidar_range * np.cos(angle)
        end_y = robot["y"] + lidar_range * np.sin(angle)

        # Simple collision check (ray vs obstacle rectangles)
        hit = False
        for obs in obstacles:
            if obs.collidepoint(end_x, end_y):
                hit = True
                break

        color = (255, 0, 0) if hit else (0, 255, 0)
        pygame.draw.line(screen, color, (robot["x"], robot["y"]), (end_x, end_y), 1)

while running:
    screen.fill((245, 245, 245))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Draw obstacles
    draw_obstacles(screen, obstacles)

    # Update robot motion
    robot = controller.update(obstacles, screen)

    # Record path (keep only last 50 points)
    path.append((int(robot["x"]), int(robot["y"])))
    if len(path) > 50:
        path.pop(0)  # remove oldest point to limit trail length

    # Draw past path (orange trail)
    for p in path:
        pygame.draw.circle(screen, (255, 150, 0), p, 2)

    # Draw LIDAR beams
    draw_lidar(screen, robot, obstacles)

    # Draw robot
    pygame.draw.circle(screen, (0, 120, 255), (int(robot["x"]), int(robot["y"])), 10)
    pygame.draw.line(
        screen,
        (0, 0, 0),
        (robot["x"], robot["y"]),
        (
            robot["x"] + 15 * np.cos(robot["theta"]),
            robot["y"] + 15 * np.sin(robot["theta"]),
        ),
        2,
    )

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
