# README #

This repository will be used for the delivery of the assignment of the Intelligent Robotics 2022-2023 course at UniPD.

# Assignment 2 #

## 2.1 Instructions to run ##

### 2.1.1 Code given for this assignment ###

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

### 2.1.2 Code developed for this assignment ###

We need to run the object detection service server:

``rosrun assignment_2 object_detections_node``

Also the object manipulation action server:

``rosrun assignment_2 object_manipulation_node``

We need to run the robot controller node:

``rosrun assignment_2 robot_controller_node``

## 2.2 Documentation about the project ##
We have created a video demonstration running all the code developed, it can be found at [this link](https://drive.google.com/file/d/1PVyt0p77H4vKBW2WnDdaWP-PGjDtrGZz/view?usp=share_link). There is also a folder called ``assignment_2/docs`` where some documents explaining our code can be found.