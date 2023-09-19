# README #

This project was developed by Javier Alquézar Alquézar, Alejandro Cano Caldero, and Jesús Moncada Ramírez enrolling _Intelligent Robotics_ at the University of Padua, Italy, during the academic year 2022-23.

## 1. Introduction
The primary goal of this project was to execute pick-and-place operations within a simulated Tiago robot environment using ROS (Robot Operating System).
This project is divided into 2 assignments.

In assignment 1 (that can be found in ``assignment1/``) we were asked to create a routine that made the Tiago navigate within the environment and, at the end of its movement, to perform object detection. This is, (1) to move the robot from a starting point to another pose (input from the command line) and (2) once the robot has reached the final pose, it detects the objects in the scene. Note that static obstacles that are part of the map (walls, shelves...) shouldn't be considered as objects.
Some rules we had to follow:
- The user should input the final pose by command line.
- The code must be implemented with an action client/server structure.
  - The action client receives the input from the user.
  - The action client calls the action server that executes all the tasks.
  - The action server sends the final list of positions of the obstacles as a result to the action client.
  - The action client also implements callbacks to the feedback of the action server and prints the task's current status in the terminal. The feedback must reflect the current status of the robot (e.g. robot stopped, robot moving, robot started obstacles detection...).

## 2. Instructions to run

First of all we have to launch the simulation and MoveIt. This command will also start ``roscore``:

``roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables``

The AprilTag detection server must be executed with:

``roslaunch tiago_iaslab_simulation apriltag.launch``

After this we need to launch the navigation stack:

``roslaunch tiago_iaslab_simulation navigation.launch``

We also need to run the provided human service node:

``rosrun tiago_iaslab_simulation human_node``

Now we have to run the TiagoController server node (developed by us in assignment 1):

``rosrun assignment_1 server_node``

We need to run the object detection service server:

``rosrun assignment_2 object_detections_node``

Also the object manipulation action server:

``rosrun assignment_2 object_manipulation_node``

We need to run the robot controller node:

``rosrun assignment_2 robot_controller_node``

## 3. Documentation
TODO

## 4. Results
TODO
