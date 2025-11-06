import pygame

def get_obstacles():
    # Each obstacle defined as (x, y, width, height)
    obstacle_data = [
        (200, 150, 80, 100),
        (400, 300, 120, 80),
        (600, 100, 60, 150)
    ]
    # Convert each tuple to a pygame.Rect
    return [pygame.Rect(x, y, w, h) for x, y, w, h in obstacle_data]    


def draw_obstacles(screen, obstacles):
    for obs in obstacles:
        pygame.draw.rect(screen, (100, 100, 100), obs)
