import numpy as np
from sensors.lidar import Lidar

class ReactiveController:
    def __init__(self, robot):
        self.robot = robot
        self.lidar = Lidar()
        self.speed = 2.0
        self.turn_rate = np.radians(10)

    def update(self, obstacles, screen=None):
        # Updated to match the latest Lidar class
        readings, _, _ = self.lidar.scan(self.robot, obstacles)  # ✅ FIXED
        x, y, theta = self.robot["x"], self.robot["y"], self.robot["theta"]

        # Detect close obstacles in the front sector
        front_sectors = readings[0:5].tolist() + readings[-5:].tolist()
        min_front = min(front_sectors)

        # Simple obstacle avoidance
        if min_front < 40:  # obstacle too close
            theta += self.turn_rate * np.random.choice([-4, -2, 2, 4])
        else:
            theta += np.random.uniform(-0.05, 0.05)
            x += self.speed * np.cos(theta)
            y += self.speed * np.sin(theta)

        # Keep robot inside window bounds
        if x <= 10 or x >= 790 or y <= 10 or y >= 590:
            theta += np.pi / 2

        # Update robot position
        self.robot["x"], self.robot["y"], self.robot["theta"] = x, y, theta
        return self.robot
