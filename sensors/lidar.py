import numpy as np
import pygame

class Lidar:
    def __init__(self, range=150, num_rays=36):
        self.range = range
        self.num_rays = num_rays

    def scan(self, robot, obstacles, screen=None):
        x, y, theta = robot["x"], robot["y"], robot["theta"]
        readings = []

        for i in range(self.num_rays):
            angle = theta + (i * 2 * np.pi / self.num_rays)
            distance = self.cast_ray(x, y, angle, obstacles, screen)
            readings.append(distance)

        return np.array(readings)

    def cast_ray(self, x, y, angle, obstacles, screen=None):
        step = 2
        for d in range(0, self.range, step):
            rx = x + d * np.cos(angle)
            ry = y + d * np.sin(angle)

            for obs in obstacles:
                ox, oy, ow, oh = obs
                if ox <= rx <= ox + ow and oy <= ry <= oy + oh:
                    return d  # hit obstacle early

            if screen:
                pygame.draw.line(screen, (180, 180, 180), (x, y), (rx, ry), 1)

        return self.range  # max range (no obstacle)
