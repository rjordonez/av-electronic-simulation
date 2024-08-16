Autonomous Exploration and Mapping Simulation
This project simulates an autonomous robot exploring and mapping an unknown environment using Pygame and Numpy. The robot navigates through a grid-based map, sensing obstacles and free spaces, and continuously updates its internal map while making decisions on where to explore next.

Features
Random Map Generation: The environment is randomly generated with a specified number of obstacles and boundary walls.
Autonomous Robot: The robot autonomously explores the environment, updating its internal map as it moves.
Pathfinding: The robot uses the A* algorithm for pathfinding to reach unexplored areas or avoid obstacles.
Pygame Visualization: The environment, robot, and its path are visualized in a Pygame window.
Installation
Clone the repository:

bash
Copy code
git clone https://github.com/yourusername/autonomous-exploration-simulation.git
cd autonomous-exploration-simulation
Install the required dependencies:

bash
Copy code
pip install numpy pygame
Usage
Run the main script to start the simulation:

bash
Copy code
python simulation.py
The simulation will run for 5 iterations, with the robot exploring a different randomly generated map in each iteration.

Code Overview
Constants:

UNKNOWN, FREE_SPACE, OBSTACLE define the different states of the map grid cells.
CELL_SIZE sets the size of each cell in the grid.
COLOR_UNKNOWN, COLOR_FREE, COLOR_OBSTACLE, COLOR_ROBOT, COLOR_PATH define the colors used for visualization.
Functions:

generateRandomMap(mapSize, numObstacles): Generates a random map with obstacles and boundary walls.
draw_map(screen, robotMap, robot_position, path): Draws the map, obstacles, and robot in the Pygame window.
initializeRobotMap(mapSize): Initializes the robot's internal map with unknown values.
identifyFrontiers(robotMap, senseData): Identifies unexplored frontiers in the map.
bestFrontier(frontiers, infoGainDict, visited): Selects the best frontier to explore based on information gain.
Classes:

Robot: Manages the robot's state, movement, map updating, and decision-making processes.
Customization
Map Size: Adjust the numRows and numCols variables to change the map size.
Obstacle Count: Modify the numObstacles variable to increase or decrease the number of obstacles.
Sensor Range: Change the sensorRange attribute in the Robot class to adjust how far the robot can sense its surroundings.
License
This project is licensed under the MIT License. See the LICENSE file for details.

Acknowledgments
This project was developed using Pygame and Numpy. Special thanks to the open-source community for providing these powerful tools.

