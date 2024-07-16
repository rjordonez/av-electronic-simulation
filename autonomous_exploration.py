import pygame
import math
import random

# Constants
WIDTH, HEIGHT = 1000, 600
ROBOT_RADIUS = 10
OBSTACLE_SIZE = 50
ACCELERATION = 0.5
FRICTION = 0.98
UI_WIDTH = 200
VISOR_RADIUS = 100
VISOR_ANGLE = 100
NUM_LINES = 20
MAX_PATH_POINTS = 500
BOUNDARY_RECT = (50, 50, WIDTH - 100, HEIGHT - 100)
GOAL_AREA = (WIDTH - 150, HEIGHT - 150, 100, 100)
class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0
        self.speed = 0
        self.path = []

    def update(self, obstacles):
        if self.speed < 2:
            self.speed += ACCELERATION
        self.move_towards_frontier(obstacles)

    def move_towards_frontier(self, obstacles):
        frontiers = self.detect_frontiers(obstacles)
        if not frontiers:
            self.angle += random.choice([-10, 10])
        else:
            nearest_frontier = min(frontiers, key=lambda f: math.dist((self.x, self.y), f))
            self.angle = math.degrees(math.atan2(nearest_frontier[1] - self.y, nearest_frontier[0] - self.x))

        next_x = self.x + self.speed * math.cos(math.radians(self.angle))
        next_y = self.y + self.speed * math.sin(math.radians(self.angle))
        
        if self.collides_with_obstacles(next_x, next_y, obstacles) or not self.is_within_bounds(next_x, next_y):
            # Turn around if stuck
            self.angle += 180
            self.angle = self.angle % 360  # Ensure angle stays within 0-360 degrees
        else:
            self.x = next_x
            self.y = next_y
            self.path.append((self.x, self.y))
            if len(self.path) > MAX_PATH_POINTS:
                self.path.pop(0)
        self.speed *= FRICTION

    def detect_frontiers(self, obstacles):
        frontiers = []
        for i in range(-VISOR_ANGLE // 2, VISOR_ANGLE // 2 + 1, 5):
            theta = math.radians(self.angle + i)
            x = self.x + VISOR_RADIUS * math.cos(theta)
            y = self.y + VISOR_RADIUS * math.sin(theta)
            if self.is_within_bounds(x, y) and not any(ob.circle_collides(x, y, ROBOT_RADIUS) for ob in obstacles):
                frontiers.append((x, y))
        return frontiers

    def is_within_bounds(self, x, y):
        return BOUNDARY_RECT[0] <= x <= BOUNDARY_RECT[0] + BOUNDARY_RECT[2] and BOUNDARY_RECT[1] <= y <= BOUNDARY_RECT[1] + BOUNDARY_RECT[3]

    def collides_with_obstacles(self, x, y, obstacles):
        for obstacle in obstacles:
            if obstacle.circle_collides(x, y, ROBOT_RADIUS):
                return True
        return False

    def draw(self, screen, obstacles):
        pygame.draw.circle(screen, (0, 0, 255), (int(self.x), int(self.y)), ROBOT_RADIUS, 0)
        if len(self.path) > 1:
            pygame.draw.lines(screen, (0, 255, 0), False, [(int(pos[0]), int(pos[1])) for pos in self.path], 2)
        for i in range(-VISOR_ANGLE // 2, VISOR_ANGLE // 2 + 1, 5):
            theta = math.radians(self.angle + i)
            end_x = self.x + VISOR_RADIUS * math.cos(theta)
            end_y = self.y + VISOR_RADIUS * math.sin(theta)
            color = (255, 0, 0) if any(ob.line_intersects(self.x, self.y, end_x, end_y) for ob in obstacles) else (255, 255, 255)
            pygame.draw.line(screen, color, (self.x, self.y), (end_x, end_y))

class Obstacle:
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size

    def draw(self, screen):
        pygame.draw.rect(screen, (255, 255, 255), (self.x, self.y, self.size, self.size))

    def circle_collides(self, x, y, radius):
        closest_x = max(self.x, min(x, self.x + self.size))
        closest_y = max(self.y, min(y, self.y + self.size))
        distance_x = x - closest_x
        distance_y = y - closest_y
        return (distance_x ** 2 + distance_y ** 2) < (radius ** 2)

    def line_intersects(self, x1, y1, x2, y2):
        # Simplified version, assumes rectangle obstacles
        return (self.line_intersects_side(x1, y1, x2, y2, self.x, self.y, self.x + self.size, self.y) or
                self.line_intersects_side(x1, y1, x2, y2, self.x, self.y, self.x, self.y + self.size) or
                self.line_intersects_side(x1, y1, x2, y2, self.x + self.size, self.y, self.x + self.size, self.y + self.size) or
                self.line_intersects_side(x1, y1, x2, y2, self.x, self.y + self.size, self.x + self.size, self.y + self.size))

    def line_intersects_side(self, x1, y1, x2, y2, x3, y3, x4, y4):
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0:
            return False
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
        if 0 <= ua <= 1 and 0 <= ub <= 1:
            return True
        return False

def initialize_pygame():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Robot Simulator")
    return screen, pygame.time.Clock()

def create_robot_and_obstacles():
    robot = Robot((WIDTH - UI_WIDTH) // 2, HEIGHT // 2)
    obstacles = [
        Obstacle(100, 100, OBSTACLE_SIZE),
        Obstacle(200, 200, OBSTACLE_SIZE),
        Obstacle(300, 300, OBSTACLE_SIZE),
        Obstacle(400, 400, OBSTACLE_SIZE),
        Obstacle(500, 500, OBSTACLE_SIZE)
    ]
    return robot, obstacles

def reshuffle_obstacles(robot, obstacles):
    robot.x = (WIDTH - UI_WIDTH) // 2
    robot.y = HEIGHT // 2
    robot.path = []
    for ob in obstacles:
        ob.x = random.randint(50, WIDTH - 50 - OBSTACLE_SIZE)
        ob.y = random.randint(50, HEIGHT - 50 - OBSTACLE_SIZE)

def draw(screen, robot, obstacles):
    screen.fill((0, 0, 0))
    # Draw boundary and goal area
    pygame.draw.rect(screen, (255, 255, 255), BOUNDARY_RECT, 2)  # Boundary
    pygame.draw.rect(screen, (0, 255, 0), GOAL_AREA)  # Goal area
    robot.draw(screen, obstacles)
    for obstacle in obstacles:
        obstacle.draw(screen)
    # Display position
    font = pygame.font.Font(None, 36)
    position_text = font.render(f'Position: ({int(robot.x)}, {int(robot.y)})', True, (255, 255, 255))
    screen.blit(position_text, (WIDTH - 500, 10))
    pygame.display.flip()

def main():
    screen, clock = initialize_pygame()
    robot, obstacles = create_robot_and_obstacles()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    reshuffle_obstacles(robot, obstacles)
        robot.update(obstacles)
        draw(screen, robot, obstacles)
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
