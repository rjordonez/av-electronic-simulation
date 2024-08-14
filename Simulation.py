import numpy as np
import pygame
from heapq import heappop, heappush

# Constants describing map
UNKNOWN = -1
FREE_SPACE = 0
OBSTACLE = 1
CELL_SIZE = 20  # Size of each cell in the grid

# Pygame Colors
COLOR_UNKNOWN = (169, 169, 169)  # Gray
COLOR_FREE = (255, 255, 255)  # White
COLOR_OBSTACLE = (0, 0, 0)  # Black
COLOR_ROBOT = (255, 0, 0)  # Red
COLOR_PATH = (0, 0, 255)  # Blue

def generateRandomMap(mapSize, numObstacles):
    """Generates a random map with obstacles and boundary walls."""
    map = np.zeros(mapSize)
    
    # Add boundary walls (obstacles)
    map[0, :] = OBSTACLE  # Top boundary
    map[-1, :] = OBSTACLE  # Bottom boundary
    map[:, 0] = OBSTACLE  # Left boundary
    map[:, -1] = OBSTACLE  # Right boundary

    # Generate random obstacles inside the map
    obstacles = set()
    while len(obstacles) < numObstacles:
        obstacle = (np.random.randint(1, mapSize[0] - 1), np.random.randint(1, mapSize[1] - 1))  # Ensure obstacles are not placed on the boundaries
        if obstacle != (0, 0) and obstacle != (mapSize[0] - 1, mapSize[1] - 1):
            obstacles.add(obstacle)
    for obstacle in obstacles:
        map[obstacle] = OBSTACLE

    return map


def draw_map(screen, robotMap, robot_position, path):
    """Draws the map, obstacles, and robot in the Pygame window."""
    for row in range(robotMap.shape[0]):
        for col in range(robotMap.shape[1]):
            rect = pygame.Rect(col * CELL_SIZE, row * CELL_SIZE, CELL_SIZE, CELL_SIZE)
            if robotMap[row, col] == UNKNOWN:
                pygame.draw.rect(screen, COLOR_UNKNOWN, rect)
            elif robotMap[row, col] == FREE_SPACE:
                pygame.draw.rect(screen, COLOR_FREE, rect)
            elif robotMap[row, col] == OBSTACLE:
                pygame.draw.rect(screen, COLOR_OBSTACLE, rect)

    # Draw the robot path
    for pos in path:
        pygame.draw.circle(screen, COLOR_PATH, (int(pos[1] * CELL_SIZE + CELL_SIZE / 2), int(pos[0] * CELL_SIZE + CELL_SIZE / 2)), 4)

    # Draw the robot
    pygame.draw.circle(screen, COLOR_ROBOT, (int(robot_position[1] * CELL_SIZE + CELL_SIZE / 2), int(robot_position[0] * CELL_SIZE + CELL_SIZE / 2)), 6)

def initializeRobotMap(mapSize):
    """Initializes the robot map with unknown values"""
    return np.full(mapSize, UNKNOWN)

class Robot:
    def __init__(self, startPosition):
        self.position = np.array(startPosition, dtype=float)
        self.path = [startPosition]
        self.sensorRange = 2  # Change to determine how big the sensor radius is
        self.step_size = 1  # Step size for continuous movement
        self.collision_threshold = 0.5  # Threshold distance to consider a collision
        self.globalSenseDict = {}
        self.infoGainDict = {}
        self.visited = set()
        self.iteration_positions = [startPosition]  # Track position at each iteration
        self.max_row = None
        self.max_col = None

    def isValidPosition(self, newPosition, map):
        """Checks if position is valid based on distance to obstacles and map boundaries"""
        if newPosition[0] < 0 or newPosition[0] >= map.shape[0] or newPosition[1] < 0 or newPosition[1] >= map.shape[1]:
            return False
        discrete_position = np.floor(newPosition).astype(int)
        if map[discrete_position[0], discrete_position[1]] == OBSTACLE:
            return False
        return True

    def expandMapIfNeeded(self, robotMap, map):
        """Expands the map size if the robot is at the boundary, but not beyond the actual map size"""
        expanded = False
        rows, cols = robotMap.shape
        actual_rows, actual_cols = map.shape
        new_robot_map = robotMap.copy()

        if self.position[0] >= rows - 1 and rows < actual_rows:
            new_robot_map = np.vstack((new_robot_map, np.full((1, cols), UNKNOWN)))
            expanded = True
        if self.position[1] >= cols - 1 and cols < actual_cols:
            new_robot_map = np.hstack((new_robot_map, np.full((new_robot_map.shape[0], 1), UNKNOWN)))
            expanded = True

        if expanded:
            print(f"Map expanded to size: {new_robot_map.shape}")
            robotMap = new_robot_map

        return robotMap

    def updateMap(self, map, robotMap):
        """Senses the environment around the robot to update RobotMap's grid knowledge (free space, obstacles, and unknown)"""
        currSenseDict = {}
        angle_resolution = 360  # Full resolution to check 360 degrees around the robot
        for angle in np.linspace(0, 2 * np.pi, angle_resolution):  # Checks 360 degrees around the robot
            for r in np.arange(0, self.sensorRange + self.step_size, self.step_size):  # Checks distance increments of sensorRange
                pos_to_check = self.position + r * np.array([np.cos(angle), np.sin(angle)])
                discrete_pos_to_check = tuple(np.round(pos_to_check).astype(int))  # Use np.round for better accuracy
                if 0 <= discrete_pos_to_check[0] < map.shape[0] and 0 <= discrete_pos_to_check[1] < map.shape[1]:  # If in-bounds, check position
                    # Expand robotMap if necessary
                    robotMap = self.expandMapIfNeeded(robotMap, map)
                    if robotMap[discrete_pos_to_check[0], discrete_pos_to_check[1]] == UNKNOWN:  # If unknown on robotMap
                        if map[discrete_pos_to_check[0], discrete_pos_to_check[1]] == OBSTACLE:  # Check actual map if obstacle
                            robotMap[discrete_pos_to_check[0], discrete_pos_to_check[1]] = OBSTACLE  # Update robotMap
                            currSenseDict[discrete_pos_to_check] = 'obstacle'  # Update Local Dict
                            self.globalSenseDict[discrete_pos_to_check] = 'obstacle'  # Update globalSenseDict for obstacle
                            break  # Stop sensing further in this direction
                        else:  # Or check if free
                            robotMap[discrete_pos_to_check[0], discrete_pos_to_check[1]] = FREE_SPACE
                            currSenseDict[discrete_pos_to_check] = 'free'
                            self.globalSenseDict[discrete_pos_to_check] = 'free'  # Update globalSenseDict for free space
                    else:  # If already known on robotMap
                        if robotMap[discrete_pos_to_check[0], discrete_pos_to_check[1]] == OBSTACLE:
                            currSenseDict[discrete_pos_to_check] = 'obstacle'
                            self.globalSenseDict[discrete_pos_to_check] = 'obstacle'  # Update globalSenseDict for obstacle
                            break  # Stop sensing further in this direction
                        else:
                            currSenseDict[discrete_pos_to_check] = 'free'
                            self.globalSenseDict[discrete_pos_to_check] = 'free'  # Update globalSenseDict for free space
                else:
                    # Update max boundaries when out of bounds detected
                    if discrete_pos_to_check[0] >= map.shape[0]:
                        self.max_row = map.shape[0] - 1
                    if discrete_pos_to_check[1] >= map.shape[1]:
                        self.max_col = map.shape[1] - 1

        return currSenseDict, robotMap

    def updateScores(self, robotMap):
        """Senses the environment around the robot to update RobotMap's grid knowledge (information gain)"""
        angle_resolution = 360  # Full resolution to check 360 degrees around the robot
        current_position = tuple(np.round(self.position).astype(int))  # Determine the current position once

        # Clear infoGainDict but preserve penalties for visited positions
        self.infoGainDict = {pos: -1000 for pos in self.visited}  # Apply heavy penalty to visited positions

        for angle in np.linspace(0, 2 * np.pi, angle_resolution):  # Checks 360 degrees around the robot
            for r in np.arange(0, self.sensorRange + self.step_size, self.step_size):  # Checks distance increments of sensorRange
                pos_to_update = self.position + r * np.array([np.cos(angle), np.sin(angle)])
                discrete_pos_to_update = tuple(np.round(pos_to_update).astype(int))  # Use np.round for better accuracy

                if 0 <= discrete_pos_to_update[0] < robotMap.shape[0] and 0 <= discrete_pos_to_update[1] < robotMap.shape[1]:  # If in-bounds, check position
                    if robotMap[discrete_pos_to_update[0], discrete_pos_to_update[1]] == FREE_SPACE:
                        if discrete_pos_to_update not in self.infoGainDict:
                            self.infoGainDict[discrete_pos_to_update] = 0
                        self.infoGainDict[discrete_pos_to_update] += 1

    def move(self, newPos, map, robotMap):
        newPos = np.array(newPos, dtype=float)
        while not np.array_equal(self.position, newPos):  # While goal hasn't been reached
            direction = np.sign(newPos - self.position)
            newPosition = self.position + direction * self.step_size
            if self.isValidPosition(newPosition, map):
                self.position = newPosition
                self.path.append(newPosition.tolist())
                self.visited.add(tuple(newPosition))
                robotMap = self.expandMapIfNeeded(robotMap, map)
                senseData, robotMap = self.updateMap(map, robotMap)  # Update map while moving
                self.updateScores(robotMap)  # Update scores while moving
                
                # Redraw the map and robot position to visualize the movement
                screen.fill(COLOR_UNKNOWN)
                draw_map(screen, robotMap, self.position, self.path)
                pygame.display.flip()
                
                # Control movement speed
                pygame.time.delay(100)  # 100 ms delay between each movement step
                
            else:
                print(f"Movement blocked at: {newPosition}")
                break
        return robotMap

    def findPathAndGo(self, goal, map, robotMap):
        current_goal = goal
        while True:
            path = a_star(robotMap, self.position, current_goal, self.step_size)
            if path:
                print(f"Path found to {current_goal}: {path}")
                for pos in path:
                    robotMap = self.move(pos, map, robotMap)
                    if np.array_equal(pos, current_goal):
                        return robotMap
                    # If we find an obstacle while moving, replan the path
                    senseData, robotMap = self.updateMap(map, robotMap)
                    if tuple(pos) in self.globalSenseDict and self.globalSenseDict[tuple(pos)] == 'obstacle':
                        print(f"Obstacle encountered at {pos}. Replanning path.")
                        break
            else:
                print(f"No path found to {current_goal}")
                break
        return robotMap

def identifyFrontiers(robotMap, senseData):
    """Identifies frontiers in the robot map based on current position"""
    frontiers = []
    currPosition = tuple(robot.position)
    for key in senseData.keys():
        if senseData[key] == 'free' and key != currPosition:
            frontiers.append(key)
    return frontiers

def bestFrontier(frontiers, infoGainDict, visited):
    """Selects the frontier with the highest information gain"""
    best_gain = float('-inf')
    best_frontier = None
    slight_punishment = -200  # Slight punishment for visited frontiers

    all_visited = True  # Flag to check if all frontiers have been visited

    for frontier in frontiers:
        if frontier in visited:
            gain = infoGainDict.get(frontier, float('-inf')) + slight_punishment  # Apply slight punishment for visited frontiers and update dictionary
            infoGainDict[frontier] = gain
        else:
            all_visited = False
            gain = infoGainDict.get(frontier, float('-inf'))

        if gain > best_gain:
            best_gain = gain
            best_frontier = frontier

    # If all frontiers are visited, apply a slight punishment but still return the best frontier
    if all_visited:
        for frontier in frontiers:
            visited.remove(frontier)  # So that it doesn't punish it with -1000
    return best_frontier

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def a_star(map, pos, goal, stepSize=0.25):
    def get_neighbors(pos):
        """Explore in 8 directions for neighboring states"""
        directions = [(stepSize, 0), (-stepSize, 0), (0, stepSize), (0, -stepSize),
                      (stepSize, stepSize), (stepSize, -stepSize), (-stepSize, stepSize), (-stepSize, -stepSize)]
        neighbors = []
        for direction in directions:
            neighbor = (pos[0] + direction[0], pos[1] + direction[1])
            if 0 <= neighbor[0] < map.shape[0] and 0 <= neighbor[1] < map.shape[1] and map[int(neighbor[0]), int(neighbor[1])] != OBSTACLE:
                neighbors.append(tuple(neighbor))
        return neighbors

    pos = tuple(pos)
    goal = tuple(goal)
    exploredSet = set()
    cameFrom = {}  # Used to reconstruct path
    gScore = {pos: 0}
    fscore = {pos: heuristic(pos, goal)}
    frontier = []
    heappush(frontier, (fscore[pos], pos))

    while frontier:  # While there is still states to explore
        current = heappop(frontier)[1]

        if heuristic(current, goal) < stepSize:  # If a current state is close enough to goal
            path = []
            while current in cameFrom:
                path.append(current)
                current = cameFrom[current]
            path.append(pos)
            path.reverse()  # Reconstruct path
            return path

        exploredSet.add(current)  # Add node to explored
        for neighbor in get_neighbors(current):  # Look through neighbors and assign gScores
            tentativeGScore = gScore[current] + heuristic(neighbor, current)
            if neighbor in exploredSet and tentativeGScore >= gScore.get(neighbor, float('inf')):  # If neighbor is not reachable or has been explored already
                continue

            if tentativeGScore < gScore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in frontier]:  # If neighbor is reachable and state or state hasn't been explored
                cameFrom[neighbor] = current
                gScore[neighbor] = tentativeGScore
                fscore[neighbor] = tentativeGScore + heuristic(neighbor, goal)
                heappush(frontier, (fscore[neighbor], neighbor))  # Add neighbor to frontier

    return False

def sampleNodes(robotMap, num_samples=5, exhausted_nodes=set()):
    """Samples nodes within the map to determine potential exploration points"""
    rows, cols = robotMap.shape
    samples = []
    attempts = 0  # To prevent infinite loops in case of few available nodes
    max_attempts = num_samples * 10  # Allow more attempts to find useful nodes

    while len(samples) < num_samples and attempts < max_attempts:
        sample = (np.random.randint(0, rows), np.random.randint(0, cols))
        if robotMap[sample] == UNKNOWN and sample not in exhausted_nodes:
            samples.append(sample)
        attempts += 1

    return samples

def pruneSampledNodes(sampled_nodes, visited):
    """Prunes sampled nodes that are on the visited list."""
    pruned_samples = []
    for sample in sampled_nodes:
        if sample not in visited:
            pruned_samples.append(sample)
    return pruned_samples

def calculateAdjustedScore(node, infoGain, robotPosition, distancePenaltyWeight, robotMap, globalSenseDict, cluster_bonus_weight=5):
    """
    Calculate the adjusted score of a point based on information gain, distance penalty, proximity to obstacles, line of sight, and future utility.
    """
    # 1. Calculate Euclidean distance between the robot and the point
    distance = np.linalg.norm(np.array(node) - np.array(robotPosition))
    
    # 2. Calculate the distance penalty
    distance_penalty = distancePenaltyWeight * distance
    
    # Proximity bonus for being near unexplored clusters
    cluster_bonus = 0
    unexplored_neighbors = 0
    for direction in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        neighbor = (node[0] + direction[0], node[1] + direction[1])
        if 0 <= neighbor[0] < robotMap.shape[0] and 0 <= neighbor[1] < robotMap.shape[1]:
            if robotMap[neighbor] == UNKNOWN:
                unexplored_neighbors += 1
    
    # Increase score if this node is near a small unexplored cluster
    if unexplored_neighbors > 0:
        cluster_bonus = cluster_bonus_weight * unexplored_neighbors
    
    # Prioritize closer unexplored nodes more strongly
    adjusted_score = (infoGain - distance_penalty + cluster_bonus)
    
    return adjusted_score


def chooseBestPoint(sampledNodes, infoGainDict, robotPosition, distancePenaltyWeight, robotMap, globalSenseDict, recent_positions, revisit_penalty_weight=500):
    """
    Choose the best point to travel to based on adjusted scores with a bias towards exploration and a mechanism to reduce oscillation.
    """
    best_point = None
    highest_score = float('-inf')

    for node in sampledNodes:
        # Get the information gain for this point
        info_gain = infoGainDict.get(node, 0)

        # Bias towards exploration by increasing the score of nodes with high info gain
        exploration_bias = 10 if robotMap[node] == UNKNOWN else 0

        # Apply a penalty if the node was visited recently
        revisit_penalty = revisit_penalty_weight if node in recent_positions else 0

        # Calculate the adjusted score considering the distance penalty, exploration bias, and revisit penalty
        adjusted_score = calculateAdjustedScore(node, info_gain + exploration_bias - revisit_penalty, robotPosition, distancePenaltyWeight, robotMap, globalSenseDict)

        # Select the point with the highest adjusted score
        if adjusted_score > highest_score:
            highest_score = adjusted_score
            best_point = node

    return best_point


# Initialize Pygame
pygame.init()

# Run 5 simulations
for sim in range(5):
    print(f"Starting simulation {sim + 1}/5")

    # Generate a random map with obstacles and define start and goal
    numRows = np.random.randint(30, 50)
    numCols = np.random.randint(30, 50)
    mapSize = (numRows, numCols)

    # Adjust numObstacles to be dependent on mapSize
    numObstacles = np.random.randint(20, min(numRows * numCols // 2, 30))  # Ensure obstacles are a smaller fraction of total cells
    map = generateRandomMap(mapSize, numObstacles)

    # Adjust the screen size based on the map size
    screen_width = mapSize[1] * CELL_SIZE
    screen_height = mapSize[0] * CELL_SIZE
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption(f"Simulation {sim + 1}/5: Autonomous Exploration and Mapping")

      # Randomize the robot's starting position
    while True:
        startRow = np.random.randint(1, numRows - 1)  # Avoiding the boundary rows
        startCol = np.random.randint(1, numCols - 1)  # Avoiding the boundary columns
        if map[startRow, startCol] != OBSTACLE:
            break

    startPosition = (startRow, startCol)

    # Initialize the robot at the randomized starting position
    robot = Robot(startPosition=startPosition)

    # Map for Robot (unknown everywhere)
    robotMap = initializeRobotMap(mapSize)
    position = startPosition

    robotMap[position] = FREE_SPACE
    robot.visited.add(position)

    # Pygame Main Loop
    running = True
    iteration = 0
    max_iterations = 1000

    clock = pygame.time.Clock()  # Initialize clock for controlling frame rate

    recent_positions = []  # List to store recent positions
    max_recent_positions = 5  # Maximum number of recent positions to remember
    num_samples = 5

    while running and iteration < max_iterations:
        iteration += 1
        print(f"\nIteration: {iteration}")
        senseData, robotMap = robot.updateMap(map, robotMap)
        robot.updateScores(robotMap)
        frontiers = identifyFrontiers(robotMap, senseData)
        sampled_nodes = sampleNodes(robotMap)
        pruned_samples = pruneSampledNodes(sampled_nodes, robot.visited)
        distancePenaltyWeight = 50
        best_point = chooseBestPoint(pruned_samples, robot.infoGainDict, robot.position, distancePenaltyWeight, robotMap, robot.globalSenseDict, recent_positions)
        exhausted_nodes = set()
        
        if best_point:
            print(f"Best point to travel to: {best_point}")
            robotMap = robot.findPathAndGo(best_point, map, robotMap)
            recent_positions.append(tuple(robot.position))  # Update recent positions
            if len(recent_positions) > max_recent_positions:
                recent_positions.pop(0)  # Maintain the size of recent positions list
        else:
            print("No valid point found. Exploring frontiers instead.")
            best_frontier_to_explore = bestFrontier(frontiers, robot.infoGainDict, robot.visited)
            print(robot.infoGainDict)
            print(f"Best frontier: {best_frontier_to_explore}")
            robotMap = robot.findPathAndGo(best_frontier_to_explore, map, robotMap)
            num_samples *= 100  # dynamically cube the number of samples each time

            exhausted_nodes.update(sampled_nodes)

        position = tuple(robot.position)
        print(f"Current position: {position}")

        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Drawing the map and robot
        screen.fill(COLOR_UNKNOWN)
        draw_map(screen, robotMap, robot.position, robot.path)
        pygame.display.flip()

        # Control the frame rate
        clock.tick(30)  # Limit to 30 frames per second

        # Final output
        print("\nGlobal Sense Dictionary:", robot.globalSenseDict)
        print("Visited Nodes:", robot.visited)

        # Calculate the number of free spaces and obstacles, excluding the boundary walls
        numObstacles = np.sum(robotMap[1:-1, 1:-1] == OBSTACLE)
        numFreeSpaces = np.sum(robotMap[1:-1, 1:-1] == FREE_SPACE)

        # Calculate the total number of cells, excluding the boundary walls
        total_cells = (map.shape[0] - 2) * (map.shape[1] - 2)

        # Calculate coverage as a percentage of the explored area
        coverage = (numObstacles + numFreeSpaces) / total_cells * 100
        print(f"Coverage: {coverage}%")

        # Check if 100% coverage is reached
        if coverage == 100:
            print("100% coverage reached. Ending simulation.")
            running = False

    pygame.time.delay(2000)  # Pause between simulations
    print(f"Ending simulation {sim + 1}/5\n")

# Quit Pygame after all simulations
pygame.quit()

print("All simulations completed.")