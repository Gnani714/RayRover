# sensors/lidar.py
import numpy as np

class Lidar:
    def __init__(self, lidar_range=150, num_rays=36, step=2):
        self.range = lidar_range
        self.num_rays = num_rays
        self.step = step

    def scan(self, robot, obstacles):
        """
        Returns:
          distances: np.array[num_rays]  (first hit or max range)
          hits: list[(x, y) or None]     (hit point for drawing)
          angles: np.array[num_rays]     (absolute angles)
        """
        x, y, theta = robot["x"], robot["y"], robot["theta"]
        distances, hits, angles = [], [], []

        for i in range(self.num_rays):
            ang = theta + (i * 2 * np.pi / self.num_rays)
            d, hit_pt = self._cast_ray(x, y, ang, obstacles)
            distances.append(d)
            hits.append(hit_pt)
            angles.append(ang)

        return np.array(distances, dtype=float), hits, np.array(angles, dtype=float)

    def _cast_ray(self, x, y, angle, obstacles):
        for d in range(0, self.range + 1, self.step):
            rx = x + d * np.cos(angle)
            ry = y + d * np.sin(angle)
            for rect in obstacles:                 # obstacles are pygame.Rect
                if rect.collidepoint(rx, ry):
                    return d, (rx, ry)
        return self.range, None
