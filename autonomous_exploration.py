import pygame
import numpy as np
import math
import time

# Initialize Pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 400, 400
GRID_SIZE = 10
CELL_SIZE = WIDTH // GRID_SIZE
LINE_WIDTH = 5
FPS = 60

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)

# Create a map with 0s representing free space and 1s representing obstacles
mapSize = (GRID_SIZE, GRID_SIZE)
map = np.zeros(mapSize)

# Add in obstacles where 1s represent obstacles
# Instead of filling entire cells, draw lines
obstacles = [
    ((3, 0), (3, 5)),
    ((0, 7), (5, 7)),
    ((2, 1), (2, 1)),  # small line at a single point
    ((5, 5), (5, 6)),
]

# Define exit point
exit_point = (9, 9)

print(map)

class Robot:
    def __init__(self, startPosition, radius=0.5):
        self.position = np.array(startPosition, dtype=float)
        self.path = [startPosition]
        self.radius = radius
        self.orientation = 0  # Initially facing right (0 degrees)
        self.image = pygame.image.load('car.png')
        self.image = pygame.transform.scale(self.image, (CELL_SIZE, CELL_SIZE // 2))  # Match width, reduce height
        self.image = pygame.transform.rotate(self.image, -90)  # Rotate to face up initially

    def move(self, distance=0.1):
        """Move in the current direction"""
        angle_rad = math.radians(self.orientation)
        direction_vector = np.array([math.cos(angle_rad), math.sin(angle_rad)])
        newPosition = self.position + direction_vector * distance

        # Check if the new position is within bounds and not an obstacle
        if self.isValidPosition(self.position, newPosition):
            self.position = newPosition
            self.path.append(newPosition)
            if len(self.path) > 3:
                self.path.pop(0)
            return True
        else:
            return False

    def set_angle(self, angle):
        """Set the robot's orientation angle"""
        self.orientation = angle

    def isValidPosition(self, start, end):
        """Checks if the path from start to end intersects with any obstacles and is within bounds"""
        # Check boundaries
        if not (0 <= end[0] < GRID_SIZE and 0 <= end[1] < GRID_SIZE):
            return False

        for obstacle_start, obstacle_end in obstacles:
            if self.line_intersection(start, end, obstacle_start, obstacle_end):
                return False
        return True

    def line_intersection(self, p1, p2, p3, p4):
        """Check if line segment p1-p2 intersects with line segment p3-p4"""
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])
        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def draw(self, screen):
        for p in self.path:
            pygame.draw.circle(screen, BLUE, (int(p[1] * CELL_SIZE + CELL_SIZE // 2), int(p[0] * CELL_SIZE + CELL_SIZE // 2)), int(self.radius * CELL_SIZE), 1)

        # Draw the car image
        rotated_image = pygame.transform.rotate(self.image, -self.orientation)
        rect = rotated_image.get_rect(center=(self.position[1] * CELL_SIZE + CELL_SIZE // 2, self.position[0] * CELL_SIZE + CELL_SIZE // 2))
        screen.blit(rotated_image, rect.topleft)

def draw_screen(screen, robot):
    screen.fill(BLACK)

    # Draw the obstacles as lines
    for start, end in obstacles:
        start_pos = (start[1] * CELL_SIZE + CELL_SIZE // 2, start[0] * CELL_SIZE + CELL_SIZE // 2)
        end_pos = (end[1] * CELL_SIZE + CELL_SIZE // 2, end[0] * CELL_SIZE + CELL_SIZE // 2)
        pygame.draw.line(screen, WHITE, start_pos, end_pos, LINE_WIDTH)

    # Draw the robot
    robot.draw(screen)

    pygame.display.flip()
    time.sleep(0.0001)

# Initialize the robot at the starting position (0, 0)
robot = Robot(startPosition=(0, 0))

# Initialize Pygame screen
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Maze Control")
clock = pygame.time.Clock()

# Main loop to handle events and update the screen
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                robot.move()  # Move in the current direction
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mouse_x, mouse_y = event.pos
            dx = mouse_x - (robot.position[1] * CELL_SIZE + CELL_SIZE // 2)
            dy = mouse_y - (robot.position[0] * CELL_SIZE + CELL_SIZE // 2)
            angle = math.degrees(math.atan2(dy, dx))
            robot.set_angle(angle)

    draw_screen(screen, robot)
    clock.tick(FPS)

pygame.quit()
