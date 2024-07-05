# -*- coding: utf-8 -*-
"""Autonomous Exploration.ipynb

Automatically generated by Colab.

Original file is located at
    https://colab.research.google.com/drive/1R-vmmORLjlQRczr0p8Ov5pvMciXpOMYK

1) Create Map
"""

import numpy as np
import matplotlib.pyplot as plt

# create a map with 0s representing free space
mapSize = (10, 10)
map = np.zeros(mapSize)

# add in obstacles where 1s represent obstacle
map[0:5, 3] = 1
map[7, 0:5] = 1
map [2, 1] = 1
map [5, 5] = 1

print(map)

"""2) Create Robot Class"""

class Robot:
    def __init__(self, startPosition):
        self.position = np.array(startPosition)
        self.path = [startPosition]
        self.sensorRange = 1  # change to determine how big the sensor radius is

    def move(self, direction):
        """Move in the four cardinal directions"""
        #senses = self.sense()

        if direction == 0:  # Up
            newPosition = self.position + [-1, 0]
        elif direction == 1:  # Down
            newPosition = self.position + [1, 0]
        elif direction == 2:  # Left
            newPosition = self.position + [0, -1]
        elif direction == 3:  # Right
            newPosition = self.position + [0, 1]

        newPosition = tuple(newPosition)

        # If move is valid, update position and path list
        if self.isValidPosition(newPosition):
            self.position = np.array(newPosition)
            self.path.append(newPosition)
            return True
        else:
            # Check why the move is invalid
            if not (0 <= newPosition[0] < mapSize[0] and 0 <= newPosition[1] < mapSize[1]):
                print("Out of bounds")
            elif map[newPosition] == 1:
                print("Run into obstacle")
            return False

    def isValidPosition(self, position):
        """Checks if position is valid"""
        # If position is within bounds
        if (position[0] >= 0 and position[0] < mapSize[0]) and (position[1] >= 0 and position[1] < mapSize[1]):
            # If position is not an obstacle
            return map[position] == 0
        return False

    def sense(self):
        """Senses the environment around the robot"""
        obstacles = []
        # Loop through each direction
        for dx in range(-self.sensorRange, self.sensorRange + 1):
            for dy in range(-self.sensorRange, self.sensorRange + 1):
              # Check if the position is within the map bounds
              PosToCheck = (self.position[0] + dx, self.position[1] + dy)
              # check if in-bounds
              if PosToCheck[0] >= 0 and PosToCheck[0] < mapSize[0] and PosToCheck[1] >= 0 and PosToCheck[1] < mapSize[1]:
                # check if obstacle
                if map[PosToCheck] == 1:
                  obstacles.append(PosToCheck)
        return obstacles

# Initialize the robot at the starting position (0, 0)
robot = Robot(startPosition=(0, 0))

"""3) Simulate Movement on Map"""

# Valid path
valid_movements = [3, 3, 1, 1, 1, 1, 1, 3, 3, 1, 3, 1, 1, 3, 3, 3, 3, 1, 2]

# Out of bound path stops at 1st index
out_of_bound_movements = [0, 3, 1, 1, 1, 1, 1, 3, 3, 1]

# Run into obstacle path, stops at 2nd index
invalid_movements = [3, 1, 1, 1, 2, 3]

obstacles = []
i = 0
for move in valid_movements:
    senses = robot.sense()
    print(f"Obstacles at {i}th iteration: ", senses)
    # for sense in senses: #need when building entire map
    #   obstacles.append(sense)
    if not robot.move(move):
        break
    i += 1
# print(obstacles)

# Doing multiple movements is wonky because it all gets added onto the robot's path list. Need to find a way to isolate each of these movement lists
# for move in out_of_bound_movements:
#     if not robot.move(move):
#         break

# for move in invalid_movements:
#     if not robot.move(move):
#         break

path = np.array(robot.path)
print(path)

plt.imshow(map, cmap='gray')
plt.plot(path[:, 1], path[:, 0], marker='o', color='r')
plt.title('Robot Path')
plt.show()

# right now just getting robot to move. eventually want to allow it to store data about free and obstacle space to build up a map

""" NEXT TIME: add more features to the robot: radius sensor, obstacle avoidance, self movement"""