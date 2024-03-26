# ENPM-661-Project 3
## A* Pathfinding for a Mobile Robot
This repository contains the implementation of the A* pathfinding algorithm designed for a mobile robot navigating a specified area with obstacles.

## Overview
The A* pathfinding algorithm is utilized to find the optimal path from a start point to a goal point within a map filled with obstacles. The project demonstrates the algorithm's ability to navigate complex environments by considering the robot's clearance, step size, and orientation.

## Features
- A* Algorithm Implementation
- Obstacle space definition with clearance consideration
- Visualization of node exploration and optimal path generation
- User input for start and goal positions, robot clearance, and step size

## Dependencies
- Python 3.x
- NumPy
- OpenCV
- google.colab.patches for visualization in Google Colab notebooks (if running in Colab)

## Running the Code
Enter the required inputs when prompted:
- Start Point Coordinates (X, Y, Θ)
- Goal Point Coordinates (X, Y, Θ)
- Clearance and robot radius
- Step size of the robot in units (1 <= L <= 10)

## Visualization
Upon successful execution, the algorithm outputs an animation showing the node exploration and the optimal path from the start to the goal point. The visualization starts only after the exploration is complete and the optimal path is found.

## Output of given inputs
In the 'output_a_star.PNG' image and 'a_star_path_finding' video file, inputs are:
- (21,23,30) for start point
- (1100,21,60) for goal point
- Clearance and Step size are 5

## Team Members
- Fazil Mammadli (mammadli@umd.edu) 120227561
- Hoang Pham (hmp61595) 120230301
