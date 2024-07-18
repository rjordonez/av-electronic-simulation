import pygame
import math
import random
from collections import defaultdict

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
GRID_SIZE = 10

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0
        self.speed = 0
        self.path = []
        self.visited = defaultdict(bool)
        self.quadrant_counts = {
            "top-left": 0,
            "top-right": 0,
            "bottom-left": 0,
            "bottom-right": 0
        }

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
            self.mark_visited()
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

    def mark_visited(self):
        radius_in_cells = int(ROBOT_RADIUS // GRID_SIZE)
        for i in range(-radius_in_cells, radius_in_cells + 1):
            for j in range(-radius_in_cells, radius_in_cells + 1):
                if math.sqrt(i**2 + j**2) * GRID_SIZE <= ROBOT_RADIUS:
                    grid_x = int((self.x // GRID_SIZE) + i)
                    grid_y = int((self.y // GRID_SIZE) + j)
                    self.visited[(grid_x, grid_y)] = True
                    self.update_quadrant_count(grid_x, grid_y)

    def update_quadrant_count(self, grid_x, grid_y):
        if grid_x < WIDTH // (2 * GRID_SIZE) and grid_y < HEIGHT // (2 * GRID_SIZE):
            self.quadrant_counts["top-left"] += 1
        elif grid_x >= WIDTH // (2 * GRID_SIZE) and grid_y < HEIGHT // (2 * GRID_SIZE):
            self.quadrant_counts["top-right"] += 1
        elif grid_x < WIDTH // (2 * GRID_SIZE) and grid_y >= HEIGHT // (2 * GRID_SIZE):
            self.quadrant_counts["bottom-left"] += 1
        elif grid_x >= WIDTH // (2 * GRID_SIZE) and grid_y >= HEIGHT // (2 * GRID_SIZE):
            self.quadrant_counts["bottom-right"] += 1

    def get_most_explored_quadrant(self):
        return max(self.quadrant_counts, key=self.quadrant_counts.get)

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
    robot.visited.clear()
    robot.quadrant_counts = {
        "top-left": 0,
        "top-right": 0,
        "bottom-left": 0,
        "bottom-right": 0
    }
    for ob in obstacles:
        ob.x = random.randint(50, WIDTH - 50 - OBSTACLE_SIZE)
        ob.y = random.randint(50, HEIGHT - 50 - OBSTACLE_SIZE)

def draw_sparse_matrix(screen, visited, opacity):
    overlay = pygame.Surface((WIDTH, HEIGHT))
    overlay.set_alpha(opacity)
    overlay.fill((0, 0, 0))
    for (grid_x, grid_y), is_visited in visited.items():
        if is_visited:
            x = grid_x * GRID_SIZE
            y = grid_y * GRID_SIZE
            pygame.draw.rect(overlay, (255, 255, 255), (x, y, GRID_SIZE, GRID_SIZE))
    screen.blit(overlay, (0, 0))

def draw(screen, robot, obstacles, sparse_matrix_opacity):
    screen.fill((0, 0, 0))
    # Draw boundary and goal area
    pygame.draw.rect(screen, (255, 255, 255), BOUNDARY_RECT, 2)  # Boundary
    pygame.draw.rect(screen, (0, 255, 0), GOAL_AREA)  # Goal area
    draw_sparse_matrix(screen, robot.visited, sparse_matrix_opacity)
    robot.draw(screen, obstacles)
    for obstacle in obstacles:
        obstacle.draw(screen)
    # Display position
    font = pygame.font.Font(None, 36)
    position_text = font.render(f'Position: ({int(robot.x)}, {int(robot.y)})', True, (255, 255, 255))
    screen.blit(position_text, (WIDTH - 500, 10))
    # Display most explored quadrant
    most_explored_text = font.render(f'Most Explored: {robot.get_most_explored_quadrant()}', True, (255, 255, 255))
    screen.blit(most_explored_text, (WIDTH - 500, 50))
    pygame.display.flip()

def main():
    screen, clock = initialize_pygame()
    robot, obstacles = create_robot_and_obstacles()

    running = True
    sparse_matrix_opacity = 50  # Low opacity
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    reshuffle_obstacles(robot, obstacles)
                elif event.key == pygame.K_s:
                    sparse_matrix_opacity = 200  # Full opacity
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_s:
                    sparse_matrix_opacity = 50  # Low opacity

        robot.update(obstacles)
        draw(screen, robot, obstacles, sparse_matrix_opacity)
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
