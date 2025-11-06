# viz/lidar_viz.py
import pygame
import math

class LidarScanner:
    def __init__(self, lidar, show_sweep=True):
        self.lidar = lidar
        self.show_sweep = show_sweep
        self.sweep_angle = 0.0
        self.sweep_speed = math.radians(180) / 30.0  # ~180°/sec at 30 FPS

    def draw(self, screen, robot, distances, hits, angles):
        x, y = robot["x"], robot["y"]

        # Rays
        for d, hit, ang in zip(distances, hits, angles):
            ex = x + d * math.cos(ang)
            ey = y + d * math.sin(ang)
            color = (255, 60, 60) if hit else (80, 200, 80)
            pygame.draw.line(screen, color, (x, y), (ex, ey), 1)
            if hit:
                pygame.draw.circle(screen, (220, 0, 0), (int(ex), int(ey)), 3)

        # Sweep line (cosmetic radar feel)
        if self.show_sweep:
            self.sweep_angle = (self.sweep_angle + self.sweep_speed) % (2 * math.pi)
            sx = x + self.lidar.range * math.cos(self.sweep_angle)
            sy = y + self.lidar.range * math.sin(self.sweep_angle)
            pygame.draw.line(screen, (50, 120, 220), (x, y), (sx, sy), 2)

        # Mini HUD (range circle)
        pygame.draw.circle(screen, (180, 180, 180), (int(x), int(y)), int(self.lidar.range), 1)
