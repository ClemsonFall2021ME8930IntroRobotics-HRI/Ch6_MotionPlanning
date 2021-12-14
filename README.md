# Ch7_MotionPlanning
## Problem Statement:
This repository contains the code for various offline motion planning algorithms. Motion planning is a term used in robotics for the process of breaking down the desired movement task into discrete motions that satisfy movement constraints and possibly optimize some aspect of the movement. It involves getting a robot to automatically determine how to move while avoiding collisions with obstacles. This repository includes the following offline motion planning algorithms:
- Grid based motion planning algorithms
  - Uniform cost search algorithm
  - Dijkstra algorithm
  - A* algorithm
- Sampling based motion planning algorithms
  - PRM Planner
  - RRT 
  - RRT*
## Dependencies:
- Opencv
- numpy
- queue
- PIL
- Coppeliasim

## Pre-requisites:
- Python programming knowledge
- Coppeliasim knowledge
- Opencv knowledge

## Simulation Environment:
In order to show how various offline motion planning algorithms works, I am also going to use the Copellasim simulation for representing these offline motion planning.  I will be using Pioneer 3-DX as the robot in the Copellasim environment . In the simulation, a camera is added, which basically shows the whole environment as an image. The main reason for the camera usage is to visualize these offline motion planning algorithms, which is understandable by everyone. The same simulation setup is used for all offline motion planning algorithms, in order to show the difference between these algorithms. 
![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/copellasim.png?raw=true)
![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/screen_floor.png?raw=true)
## Simulation Output

### Uniform Cost Search:
![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/UCS.png?raw=true)

### Dijkstra:
![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/djikstra.png?raw=true)

### A*:
![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/Astar.png?raw=true)

### PRM Planner:
![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/prm_points.png?raw=true)

![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/prm_cons.png?raw=true)

![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/prm.png?raw=true)

### RRT:
![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/RRT.png?raw=true)

### RRT*:
![alt text](https://github.com/ClemsonFall2021ME8930IntroRobotics-HRI/Ch7_MotionPlanning/blob/main/Outputimages/rrt*.png?raw=true)
