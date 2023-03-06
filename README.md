# README #

This project was developed by Javier Alquézar Alquézar, Alejandro Cano Caldero and Jesús Moncada Ramírez enrolling "Intelligent Robotics" at the University of Padova, in the academic year 2022-23.

## 1. Introduction
TODO

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
