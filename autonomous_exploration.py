import pygame
import math

# Constants
WIDTH, HEIGHT = 1000, 600
ROBOT_RADIUS = 10
OBSTACLE_SIZE = 50
ACCELERATION = 0.5
FRICTION = 0.98
UI_WIDTH = 200
VISOR_RADIUS = 100
VISOR_ANGLE = 100
NUM_LINES = 20  # Number of lines pointing outward
MAX_PATH_POINTS = 500  # Maximum number of points to store in the path

# Robot class
class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0
        self.speed = 0
        self.acceleration = 0
        self.path = []  # Use a list to store the path positions

    def draw(self, screen, obstacles):
        self.draw_heatmap(screen)  # Draw the heatmap before the robot
        pygame.draw.circle(screen, (0, 0, 255), (int(self.x), int(self.y)), ROBOT_RADIUS, 1)
        self.draw_lines(screen, obstacles)
        self.draw_visor(screen, obstacles)
        self.draw_path(screen)

    def draw_lines(self, screen, obstacles):
        for i in range(NUM_LINES):
            theta = 2.0 * math.pi * i / NUM_LINES
            x = self.x + VISOR_RADIUS * math.cos(theta)
            y = self.y + VISOR_RADIUS * math.sin(theta)
            intersect_x, intersect_y = self.check_obstacle_intersection(x, y, obstacles)
            pygame.draw.line(screen, (255, 255, 255), (self.x, self.y), (intersect_x, intersect_y))

    def draw_visor(self, screen, obstacles):
        points = [(self.x, self.y)]
        for i in range(-VISOR_ANGLE // 2, VISOR_ANGLE // 2 + 1):
            theta = math.radians(self.angle + i)
            x = self.x + VISOR_RADIUS * math.cos(theta)
            y = self.y + VISOR_RADIUS * math.sin(theta)
            intersect_x, intersect_y = self.check_obstacle_intersection(x, y, obstacles)
            points.append((intersect_x, intersect_y))
        pygame.draw.polygon(screen, (0, 0, 255, 180), points)

    def draw_path(self, screen):
        if len(self.path) > 1:
            pygame.draw.lines(screen, (0, 255, 0), False, [(int(pos[0]), int(pos[1])) for pos in self.path], 2)

    def check_obstacle_intersection(self, x, y, obstacles):
        for obstacle in obstacles:
            if obstacle.line_intersects(self.x, self.y, x, y):
                return obstacle.intersection_point(self.x, self.y, x, y)
        return x, y

    def update(self, obstacles):
        next_x = self.x + self.speed * math.cos(math.radians(self.angle))
        next_y = self.y + self.speed * math.sin(math.radians(self.angle))
        if not self.collides_with_obstacles(next_x, next_y, obstacles):
            self.x = next_x
            self.y = next_y
            self.path.append((self.x, self.y))  # Append the current position to the path
            if len(self.path) > MAX_PATH_POINTS:  # Limit the number of points in the path
                self.path.pop(0)
        self.speed *= FRICTION
        self.keep_in_bounds()

    def collides_with_obstacles(self, x, y, obstacles):
        for obstacle in obstacles:
            if obstacle.circle_collides(x, y, ROBOT_RADIUS):
                return True
        return False

    def keep_in_bounds(self):
        if self.x < 0: self.x = 0
        if self.x > WIDTH - UI_WIDTH: self.x = WIDTH - UI_WIDTH
        if self.y < 0: self.y = 0
        if self.y > HEIGHT: self.y = HEIGHT

    def draw_heatmap(self, screen):
        max_visits = max(self.path.count(pos) for pos in set(self.path))  # Get the maximum visit count
        for pos in set(self.path):
            count = self.path.count(pos)
            red = min(255, int(255 * count / max_visits))
            green = 255 - red
            color = (red, green, 0, 128)
            pygame.draw.circle(screen, color, (int(pos[0]), int(pos[1])), ROBOT_RADIUS)

# Obstacle class
class Obstacle:
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size

    def draw(self, screen):
        pygame.draw.rect(screen, (255, 255, 255), (self.x, self.y, self.size, self.size))

    def circle_collides(self, x, y, radius):
        # Check if the circle collides with the obstacle
        closest_x = max(self.x, min(x, self.x + self.size))
        closest_y = max(self.y, min(y, self.y + self.size))
        distance_x = x - closest_x
        distance_y = y - closest_y
        return (distance_x ** 2 + distance_y ** 2) < (radius ** 2)

    def line_intersects(self, x1, y1, x2, y2):
        # Check if the line intersects any of the sides of the obstacle
        return (self.line_intersects_side(x1, y1, x2, y2, self.x, self.y, self.x + self.size, self.y) or
                self.line_intersects_side(x1, y1, x2, y2, self.x, self.y, self.x, self.y + self.size) or
                self.line_intersects_side(x1, y1, x2, y2, self.x + self.size, self.y, self.x + self.size, self.y + self.size) or
                self.line_intersects_side(x1, y1, x2, y2, self.x, self.y + self.size, self.x + self.size, self.y + self.size))

    def line_intersects_side(self, x1, y1, x2, y2, x3, y3, x4, y4):
        # Calculate intersection of two lines (x1,y1)-(x2,y2) and (x3,y3)-(x4,y4)
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0:
            return False
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
        if 0 <= ua <= 1 and 0 <= ub <= 1:
            return True
        return False

    def intersection_point(self, x1, y1, x2, y2):
        # Calculate intersection point of the line (x1,y1)-(x2,y2) with the obstacle
        points = []
        sides = [(self.x, self.y, self.x + self.size, self.y),
                 (self.x, self.y, self.x, self.y + self.size),
                 (self.x + self.size, self.y, self.x + self.size, self.y + self.size),
                 (self.x, self.y + self.size, self.x + self.size, self.y + self.size)]
        for x3, y3, x4, y4 in sides:
            denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
            if denom == 0:
                continue
            ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
            ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
            if 0 <= ua <= 1 and 0 <= ub <= 1:
                ix = x1 + ua * (x2 - x1)
                iy = y1 + ua * (y2 - y1)
                points.append((ix, iy))
        if points:
            # Return the closest intersection point
            return min(points, key=lambda p: math.dist((x1, y1), p))
        return x2, y2

# Function to calculate angle
def calculate_angle(robot, mouse_pos):
    dx = mouse_pos[0] - robot.x
    dy = mouse_pos[1] - robot.y
    return math.degrees(math.atan2(dy, dx))

# Function to draw UI
def draw_ui(screen, robot):
    pygame.draw.rect(screen, (50, 50, 50), (WIDTH - UI_WIDTH, 0, UI_WIDTH, HEIGHT))
    font = pygame.font.Font(None, 36)
    direction_text = font.render(f'Direction: {robot.angle:.2f}°', True, (255, 255, 255))
    screen.blit(direction_text, (WIDTH - UI_WIDTH + 10, HEIGHT - 30))
    position_text = font.render(f'Position: ({int(robot.x)}, {int(robot.y)})', True, (255, 255, 255))
    screen.blit(position_text, (WIDTH - UI_WIDTH + 10, HEIGHT - 60))
    acceleration_text = font.render(f'Acceleration: {robot.acceleration:.2f}', True, (255, 255, 255))
    screen.blit(acceleration_text, (WIDTH - UI_WIDTH + 10, HEIGHT - 90))

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

def handle_events():
    global mouse_pos, accelerate
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
        elif event.type == pygame.MOUSEMOTION:
            mouse_pos = event.pos
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                accelerate = True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_SPACE:
                accelerate = False
    return True

def update_robot(robot, obstacles):
    robot.angle = calculate_angle(robot, mouse_pos)
    if accelerate:
        robot.acceleration = ACCELERATION
        robot.speed += ACCELERATION
    else:
        robot.acceleration = 0
    robot.update(obstacles)

def draw(screen, robot, obstacles):
    screen.fill((0, 0, 0))
    robot.draw(screen, obstacles)
    for obstacle in obstacles:
        obstacle.draw(screen)
    draw_ui(screen, robot)
    pygame.display.flip()

def main():
    screen, clock = initialize_pygame()
    robot, obstacles = create_robot_and_obstacles()
    global mouse_pos, accelerate
    mouse_pos = (WIDTH // 2, HEIGHT // 2)
    accelerate = False

    running = True
    while running:
        running = handle_events()
        update_robot(robot, obstacles)
        draw(screen, robot, obstacles)
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
