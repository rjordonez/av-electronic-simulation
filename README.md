autonomous navigation: numpy map, hard-coded movement, checks to keep robot in bounds and away from obstacles

autonomous navigation 1: added radius sensor which keeps track of obstacles near robot at each move

autonomous navigation 2: refined movement to allow for continuous movement in the discrete grid environment

autonomous navigation 3: A* path finding implemented, tracks path and detected obstacles on path, random map generator

autonomous navigation 4: implemented a simple version of frontier-based exploration choosing only the closest frontier for robot to build its own map

autonomous navigation 5: implemented a cost map for frontier-based exploration. punishes previously visited nodes encouraging forward movement. implemented metric trackers

autonomous navigation 6: introduced map assumptions by robot. robot samples points in assumed space and chooses the furthest one from the current path. traverses to that path using dynamic A*. when there are no sampled points (don't meet criteria of not being pruned), it falls back to frontier-based exploration with the cost map
