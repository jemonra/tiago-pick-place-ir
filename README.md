# TIAGo Pick-and-Place Project

This project was developed by **Javier Alquézar Alquézar**, [**Alejandro Cano Caldero**](https://github.com/AlejandroCCaldero), and [**Jesús Moncada Ramírez**](https://github.com/jemonra) while enrolling in *Intelligent Robotics* at the University of Padua, Italy, during the academic year 2022–23.

## 1. Introduction

The primary goal of this project was to execute pick-and-place operations within a simulated TIAGo robot environment using ROS (Robot Operating System).  
The project was divided into two assignments:

### Assignment 1 – Navigation and Obstacle Detection

The objective was to create a routine to make the TIAGo robot navigate to a user‑defined pose and, once there, detect movable obstacles in the environment (ignoring static map elements like walls or shelves).

Key requirements:
- The user inputs the final pose via the command line.
- The code follows an action client/server architecture:
  - The *action client* receives the user input and calls the *action server*.
  - The *action server* commands the robot to move, detects obstacles at the destination, and returns their positions.
  - The client displays feedback (robot stopped, moving, detecting obstacles, etc.) and prints the final list of detected obstacles.

### Assignment 2 – Pick-and-Place Task

Building on Assignment 1, the second stage required a complete pick‑and‑place pipeline:
- Robot navigation to pick and drop locations.
- Object detection with AprilTags.
- Object manipulation using the TIAGo arm and gripper.

The code was modularized into three main ROS nodes:
- `robot_controller_node`: Orchestrates the full sequence, from receiving the object order to navigation, detection, grasping, and placing.
- `object_detections_node`: Detects AprilTags and returns the IDs and poses of all movable objects.
- `object_manipulation_node`: Controls the robot arm to grasp or drop an object, updating the planning scene with collision objects.

## 2. System Architecture

```
├── assignment1/
│   ├── src/client.cpp
│   ├── src/server.cpp
│   └── src/obstacle_detection.cpp
└── assignment2/
    ├── src/robot_controller.cpp
    ├── src/object_detections.cpp
    ├── src/object_manipulation.cpp
    └── src/pick_place_node.cpp   (testing only)
```

### Assignment 1 Details

- Client Node (`client.cpp`)
  Prompts the user for a target pose and sends it to the `tiago_controller` action server.  
  Handles feedback and prints detected obstacles.
- Server Node (`server.cpp`)
  Interfaces with the `move_base` action server to navigate to the goal pose.  
  Upon arrival, it calls the obstacle detection service and returns the obstacle positions.
- Obstacle Detection (`obstacle_detection.cpp`)
  Compares LIDAR scan points to the map, discarding static points and clustering remaining points into movable obstacles.

### Assignment 2 Details

- Robot Controller (`robot_controller.cpp`)
  Central node calling the human service to get the pick/drop order and coordinating navigation, detection, and manipulation.
- Object Detections (`object_detections.cpp`)
  Subscribes to AprilTag detections, transforms poses to the robot frame, and returns a list of objects and poses.
- Object Manipulation (`object_manipulation.cpp`)
  Uses MoveIt to plan and execute arm motions, attaching and detaching objects in Gazebo through the `gazebo_ros_link_attacher` service.

## 3. Instructions to Run

1. Launch the simulation and MoveIt (starts `roscore`):
   ```bash
   roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables
   ```
2. Start AprilTag detection:
   ```bash
   roslaunch tiago_iaslab_simulation apriltag.launch
   ```
3. Launch the navigation stack:
   ```bash
   roslaunch tiago_iaslab_simulation navigation.launch
   ```
4. Run the human service node:
   ```bash
   rosrun tiago_iaslab_simulation human_node
   ```
5. Assignment 1 nodes:
   ```bash
   rosrun assignment_1 server_node
   ```
6. Assignment 2 nodes:
   ```bash
   rosrun assignment_2 object_detections_node
   rosrun assignment_2 object_manipulation_node
   rosrun assignment_2 robot_controller_node
   ```

## 4. Results

### Assignment 1
- Navigation: The TIAGo successfully reached user-specified poses while continuously publishing feedback on its status.
- Obstacle detection: The system correctly identified movable obstacles by comparing real-time LIDAR scans with the static map and clustering remaining points.

### Assignment 2
- Integrated pipeline: The robot autonomously retrieved the ordered list of objects, navigated to the pick table, detected and grasped each target, and placed them on the designated tables.
- Robust manipulation: Added immovable and movable collision objects to the MoveIt planning scene, enabling safe and repeatable grasping and placing operations.

These results demonstrate a fully functional ROS-based pipeline combining navigation, perception, and manipulation in a simulated TIAGo robot.

## 5. Documentation

For a detailed explanation of the implementation, refer to the two reports:
- [Report for assignment 1](assignment_1/docs/Report.pdf)
- [Report for assignment 2](assignment_2/docs/Report.pdf)

They include code structure, algorithms, and design decisions for each assignment.
