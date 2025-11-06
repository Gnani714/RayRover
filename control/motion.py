import numpy as np
from sensors.lidar import Lidar

class MotionController:
    def __init__(self, robot):
        self.robot = robot
        self.lidar = Lidar()
        self.speed = 2.0
        self.turn_rate = np.radians(10)

    def update(self, obstacles, screen=None):
        readings = self.lidar.scan(self.robot, obstacles, screen)
        x, y, theta = self.robot["x"], self.robot["y"], self.robot["theta"]

        # Find if there’s something too close in front
        front_sectors = readings[0:5].tolist() + readings[-5:].tolist()
        min_front = min(front_sectors)

        # Simple avoidance behavior
        if min_front < 40:  # obstacle close ahead
            theta += self.turn_rate * np.random.choice([-4, -2, 2, 4])
        else:
            theta += np.random.uniform(-0.05, 0.05)
            x += self.speed * np.cos(theta)
            y += self.speed * np.sin(theta)

        # Keep robot within screen
        if x <= 10 or x >= 790 or y <= 10 or y >= 590:
            theta += np.pi / 2

        self.robot["x"], self.robot["y"], self.robot["theta"] = x, y, theta
        return self.robot
