#include <ros/ros.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_1/TiagoControllerAction.h>
#include <assignment_2/ObjectManipulationAction.h>
#include <assignment_2/object_detections.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<assignment_1::TiagoControllerAction> TiagoControllerClient;
typedef actionlib::SimpleActionClient<assignment_2::ObjectManipulationAction> ObjectManipulationClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryClient;

class RobotController
{
    private:
    
    // Node handle for this class
	ros::NodeHandle nh_;
    
    // Client for service /human_objects_srv
    ros::ServiceClient humanService_;

    // Client for action /tiago_controller (created by us in assignment 1)
    TiagoControllerClient tiagoControllerAction_;

    // Client for service /object_detections (created by us)
	ros::ServiceClient objectDetectionService_;

    // Client for action /object_manipulation (created by us)
    ObjectManipulationClient objectManipulationAction_;

    // Client for action /head_controller/follow_joint_trajectory
    boost::shared_ptr<FollowJointTrajectoryClient> headClient_;

    // Client for action /torso_controller/follow_joint_trajectory
    boost::shared_ptr<FollowJointTrajectoryClient> torsoClient_;

    public:

    //Constructor
    RobotController() : tiagoControllerAction_("tiago_controller", true),
                        objectManipulationAction_("object_manipulation", true)
    {
        humanService_ = nh_.serviceClient<tiago_iaslab_simulation::Objs>("human_objects_srv");
        humanService_.waitForExistence();
        ROS_INFO("Service server /human_objects_srv is ready");

	    tiagoControllerAction_.waitForServer(); //will wait infinite time
        ROS_INFO("Action server /tiago_controller is ready");

        objectManipulationAction_.waitForServer(); //will wait infinite time
        ROS_INFO("Action server /object_manipulation is ready");

        objectDetectionService_ = nh_.serviceClient<assignment_2::object_detections>("object_detections");
        objectDetectionService_.waitForExistence();
        ROS_INFO("Service server /object_detections is ready");

        headClient_.reset(new FollowJointTrajectoryClient("/head_controller/follow_joint_trajectory"));
        while(!headClient_->waitForServer(ros::Duration(2.0)) && ros::ok())
        {
            ROS_DEBUG("\tWaiting for the head action server to come up...");
        }
        ROS_INFO("Action server for robot head is ready");

        torsoClient_.reset(new FollowJointTrajectoryClient("/torso_controller/follow_joint_trajectory"));
        while(!torsoClient_->waitForServer(ros::Duration(2.0)) && ros::ok())
        {
            ROS_DEBUG("\tWaiting for the torso action server to come up...");
        }
        ROS_INFO("Action server for robot torso is ready");

        // Request human_node the order of objects
        tiago_iaslab_simulation::Objs humanRequestResponse;
        humanRequestResponse.request.ready = true;
        humanRequestResponse.request.all_objs = true;

        std::vector<int> ids;

        ROS_INFO("Calling human_objects_srv to receive objects...");
        if(humanService_.call(humanRequestResponse)) {
            ROS_INFO("\tResponse received from human_objects_srv");
            ids = humanRequestResponse.response.ids;
            ROS_INFO("\tObjects order: %d, %d, %d", ids.at(0), ids.at(1), ids.at(2));
        } else {
            ROS_ERROR("\tError calling human_objects_srv");
            exit(EXIT_FAILURE);
        }

        // For every object received (in order) perform the operations
        for (int id: ids)
        {
            ROS_INFO("Doing object %d", id);
            doObject(id);
        }
    }

    /***********************
     * MOVE ROBOT
    ***********************/

    /**
     * Returns the quaternion corresponding to an angle.
     *
     * Creates the quaternion corresponding to the angle specified, in the
     * z-axis, counterclockwise. This function is usually used to specify the
     * robot orientation when moving in the plane. The angle must be in radians.
     *
     * @param theta Angle, z-axis, counterclockwise, in radians.
     *
     * @return Quaternion corresponding to theta.
    */
    geometry_msgs::Quaternion zRotation(double theta)
    {
        geometry_msgs::Quaternion q;
        q.x = 0;
        q.y = 0;
        q.z = sin(theta/2);
        q.w = cos(theta/2);
        return q;
    }

    /** Moves the robot to a pose and waits until it arrives.
     *
     * Using the tiago_controller action moves the robot to the pose specified
     * in the arguments (with respecto to map frame) and waits for the action
     * server to send the result. We are not controlling the feedback messages
     * with other functions.
     *
     * @param x Coordinate x of the destination pose position.
     * @param y Coordinate y of the destination pose position.
     * @param z Coordinate z of the destination pose position.
     * @param o_x Coordinate x of the destination pose orientation.
     * @param o_y Coordinate y of the destination pose orientation.
     * @param o_z Coordinate z of the destination pose orientation.
     * @param o_w Coordinate w of the destination pose orientation.
     *
     * @return true if the destination pose was reached; false otherwise.
    */
    bool moveRobot(double x, double y, double z, double o_x, double o_y, double o_z, double o_w)
    {
        ROS_INFO("Moving robot to (%4.2f, %4.2f, %4.2f; %4.2f, %4.2f, %4.2f, %4.2f)...", x, y, z, o_x, o_y, o_z, o_w);
        assignment_1::TiagoControllerGoal goal;
        goal.obstacleDetection = false; // do not perform obstacle detection
        goal.position.target_pose.header.frame_id = "map";

        goal.position.target_pose.pose.position.x = x;
        goal.position.target_pose.pose.position.y = y;
        goal.position.target_pose.pose.position.z = z;

        goal.position.target_pose.pose.orientation.x = o_x;
        goal.position.target_pose.pose.orientation.y = o_y;
        goal.position.target_pose.pose.orientation.z = o_z;
        goal.position.target_pose.pose.orientation.w = o_w;

        tiagoControllerAction_.sendGoalAndWait(goal); // waits for the robot to move

        if (tiagoControllerAction_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("\tRobot sucessfully moved");
            return true;
        } else {
            ROS_ERROR("\tRobot couldn't move");
            return false;
        }
    }

    /**
     * Moves the robot to a pose and waits until it arrives.
     *
     * It is the same version as the function above but the orientation is
     * passed with a quaternion message.
     *
     * @param x Coordinate x of the destination pose position.
     * @param y Coordinate y of the destination pose position.
     * @param z Coordinate z of the destination pose position.
     * @param q Quaternion with the destination pose orientation.
    */
    bool moveRobot(double x, double y, double z, geometry_msgs::Quaternion q)
    {
        moveRobot(x, y, z, q.x, q.y, q.z, q.w);
    }

    /**
     * Moves the robot to an initial point, where all objects can be reached.
     *
     * This function moves the robot to an initial point in the first room where
     * all objects can be caught and all positions in this room can be reached.
     * If not moving to the initial point first, the robot can get stuck going
     * to the pick table.
     *
     * @return void.
    */
    bool moveToInitialPoint()
    {
        return moveRobot(8.5, 0, 0, zRotation(3.0*M_PI_2));
    }

    /**
     * Moves the robot to the corridor with some orientation.
     *
     * Moves the robot to the corridor between the pick table room and the put
     * tables room. An orientation must be provided (w.r.t. z-axis,
     * counterclockwise) to speed up the movementof the robot during the whole
     * process.
     *
     * @param theta Orientation of the robot (z-axis, counterclockwise).
     *
     * @return void.
    */
    bool moveToCorridor(double theta)
    {
        return moveRobot(9.50, -4.2, 0, zRotation(theta));
    }
    
    /***********************
     * CATCHING-DROPPING OBJECTS
    ***********************/

    /**
     * Calls the object_manipulation action server to catch an object of the
     * table with the robot arm.
     *
     * Catch a object specified by its id among all the other objects detected
     * by the camera. The mode specified in the action is 0 because it is a
     * catch action. The action server doesn't know if other object is already
     * caught by the gripper, this information must be known by the one who
     * calls this function.
     *
     * @param catchId Id of the object that must be caught by the robot.
     * @param detectedObjects All the other objects detected by the camera in
     * the table.
     *
     * @returns true if the object was successfully caught; false otherwise.
    */
    bool objectManipulationCatch(int catchId, std::vector<assignment_2::ObjectWithTag> detectedObjects)
    {
        ROS_INFO("Calling object_manipulation action to CATCH object id=%d...", catchId);
        
        // Check if object with id=catchId exists in detectedObjects
        bool found = false;
        for (assignment_2::ObjectWithTag o : detectedObjects)
        {
            if (o.id == catchId)
                found = true;
        }
        if (!found)
        {
            ROS_ERROR("\tObject with id=%d is not in detected objects", catchId);
            return false;
        }

        // Prepare the goal
        assignment_2::ObjectManipulationGoal goal;
        goal.mode = 0; // CATCH mode
        goal.catch_id = catchId;
        goal.objects = detectedObjects;

        // Send the goal
        objectManipulationAction_.sendGoalAndWait(goal);

        if (objectManipulationAction_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("\tRobot successfully caught object with id %d", catchId);
            return true;
        } else {
            ROS_INFO("\tRobot was not able to catch object with id %d", catchId);
            return false;
        }
    }

    /**
     * Calls the object_manipulation action server to drop an object into the
     * drop table.
     *
     * Drops the object currently caught by the gripper on a table. The mode
     * specified in the action is 1 because it is a drop action. The action
     * server doesn't know if other object is already caught by the gripper,
     * this information must be known by the one who calls this function.
     *
     * @param id Id of the object that must be dropped, that must be currently
     * caught by the arm.
     * @returns true if the object was successfully caught; false otherwise.
     */
    bool objectManipulationDrop(int id)
    {
        ROS_INFO("Calling object_manipulation action to DROP object id=%d...", id);

        assignment_2::ObjectManipulationGoal goal;
        goal.mode = 1; // DROP mode
        goal.catch_id = id;

        objectManipulationAction_.sendGoalAndWait(goal);

        if (objectManipulationAction_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("\tRobot successfully dropped object");
            return true;
        } else {
            ROS_INFO("\tRobot was not able to drop object");
            return false;
        }
    }

    /**
     * Perform the needed operations to catch an object of the table.
     *
     * Calls the object_detections server and the object_manipulation action in
     * order to (1) detect all the objects with tags from the camera and (2)
     * perform the robot operation of catching the object.
     *
     * @param id Id of the object to catch
     *
     * @return true if the operation was successfully done; false otherwise.
    */
    bool detectAndCatchObject(int id)
    {
        std::vector<assignment_2::ObjectWithTag> detectedObjects = objectDetectionsGet();
        return objectManipulationCatch(id, detectedObjects);
    }

    /***********************
     * DETECTING OBJECTS
    ***********************/

    /**
     * Calls the service object_detections to get the april tags objects
     * detected by the camera.
     *
     * Calls the service created by us object_detections to get a vector of
     * ObjectWithTag. A ObjectWithTag contains an id (the id of the detected
     * tag) and a pose. All the poses are in the robot arm reference frame,
     * base_footprint.
     *
     * @return List of objects with a tag detected by the camera.
    */
    std::vector<assignment_2::ObjectWithTag> objectDetectionsGet()
    {
        ROS_INFO("Calling object_detections service to get tags...");
        assignment_2::object_detections detection;

        if(objectDetectionService_.call(detection)) // call the obstacle detection server
        {
            ROS_INFO("\tCorrectly called object_detections");
        }
        else // ERROR with obstacle_detection server
        {  
            ROS_ERROR("\tFailed to call service object_detections");
        }

        return detection.response.objects;
    }

    /***********************
     * MOVING HEAD
    ***********************/

    /**
     * Generate a simple trajectory with one waypoint to move TIAGo's head.
     * Simply define the goal's parameters.
     *
     * @param goal Goal to be filled.
     * @param rotation Head rotation to be achieved.
     *
     * @return void.
    */ 
    void waypoints_head_goal(control_msgs::FollowJointTrajectoryGoal& goal, double rotation)
    {
        // The joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("head_1_joint");
        goal.trajectory.joint_names.push_back("head_2_joint");
    
        // One waypoint in this goal trajectory
        goal.trajectory.points.resize(1);
    
        // First trajectory point
        // Positions
        int index = 0;
        goal.trajectory.points[index].positions.resize(2);
        goal.trajectory.points[index].positions[0] = 0.0;
        goal.trajectory.points[index].positions[1] = rotation;
        
        // Velocities
        goal.trajectory.points[index].velocities.resize(2);
        for (int j = 0; j < 2; ++j)
        {
            goal.trajectory.points[index].velocities[j] = 1.0;
        }

        // To be reached 2 second after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
    }

    /**
     * Moves the robot's head to achieve a rotation.
     *
     * Calls an action server to move the robot's head with a provided rotation.
     *
     * @param rotation Rotation provided to the robot's head.
     *
     * @return true if the head was successfully moved; false otherwise
    */
    bool moveHead(double rotation)
    {
        ROS_INFO("Moving head to rotation %4.2f", rotation);

        // Generates the goal for the TIAGo's head
        control_msgs::FollowJointTrajectoryGoal head_goal;
        waypoints_head_goal(head_goal, rotation);

        // Sends the command to start the given trajectory 1s from now
        head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

        ROS_INFO("\tSending goal...");
        headClient_->sendGoalAndWait(head_goal);

        if (headClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("\tRobot successfully moved head");
            return true;
        } else {
            ROS_INFO("\tRobot was not able to move head");
            return false;
        }
    }

    /***********************
     * MOVING TORSO
    ***********************/

    /**
     * Generate a simple trajectory with one waypoint to move TIAGo's torso.
     * Simply define the goal's parameters.
     *
     * @param goal Goal to be filled.
     * @param rotation Torso position to be achieved.
     *
     * @return void.
    */ 
    void waypoints_torso_goal(control_msgs::FollowJointTrajectoryGoal& goal, double position)
    {
        // The joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("torso_lift_joint");
    
        // One waypoint in this goal trajectory
        goal.trajectory.points.resize(1);
    
        // Positions
        int index = 0;
        goal.trajectory.points[index].positions.resize(1);
        goal.trajectory.points[index].positions[0] = position;

        // Velocities
        goal.trajectory.points[index].velocities.resize(1);
        goal.trajectory.points[index].velocities[0] = 1.0;

        // To be reached 2 second after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(2.0);
    }

    /**
     * Moves the robot's torso to achieve a height.
     *
     * Calls an action server to move the robot's torso with a provided height.
     *
     * @param rotation Extension of the robot's torso.
     *
     * @return true if the torso was successfully moved; false otherwise
    */
    bool moveTorso(double position)
    {
        ROS_INFO("Moving torso to position %4.2f", position);

        // Generates the goal for the TIAGo's torso
        control_msgs::FollowJointTrajectoryGoal torso_goal;
        waypoints_torso_goal(torso_goal, position);

        // Sends the command to start the given trajectory 1s from now
        torso_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        ROS_INFO("\tSending goal...");
        torsoClient_->sendGoalAndWait(torso_goal);

        if (torsoClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("\tRobot successfully moved torso");
            return true;
        } else {
            ROS_WARN("\tRobot was not able to move torso (it could have moved)");
            return false;
        }
    }

    /** Moves an object from the pick table to its drop table.
     *
     * Performs the necessary operations to move one movable object from the
     * pick table to its drop table. To achieve this, calls function to move the
     * robot, move the robot's head, move the robot's torso and move the robot's
     * arm and gripper catching and dropping objects.
     *
     *
     * @param id Id of the object that must be moved
     * @return void
    */
    void doObject(int id)
    {   
        moveToInitialPoint();
        if (id == 1) // blue hexagon
        {
            moveRobot(8.15, -2.10, 0, zRotation(3.0*M_PI_2)); // Move to catch table

            moveTorso(0.35);
            moveHead(-0.53);
            detectAndCatchObject(id);
            
            moveRobot(8, -1, 0, zRotation(0)); // Needed to avoid getting stuck
            moveToCorridor(0);
            moveRobot(12.5, -1.10, 0, zRotation(M_PI_2)); // Move to drop table
            
            objectManipulationDrop(id);

            moveToCorridor(M_PI);
        }
        else if (id == 2) // green triangle
        {
            moveRobot(7.80, -4, 0, zRotation(M_PI_2)); // Move to catch table
            
            //moveTorso(0.35);
            moveHead(-0.53);
            detectAndCatchObject(id);
            
            moveToCorridor(0);
            moveRobot(11.45, -1.10, 0, zRotation(M_PI_2)); // Move to drop table
            
            objectManipulationDrop(id);

            moveToCorridor(M_PI);
        }
        else if (id == 3) // red cube
        {
            moveRobot(7.45, -2, 0, zRotation(3.0*M_PI_2)); // Move to catch table
            
            moveTorso(0.35);
            moveHead(-0.53);
            detectAndCatchObject(id);

            moveToCorridor(0);
            moveRobot(10.5, -1.10, 0, zRotation(M_PI_2)); // Move to drop table
            
            objectManipulationDrop(id);

            moveToCorridor(M_PI);
        }
        else
        {
            ROS_ERROR("Not recognized object with id=%d", id);
        }
        moveToInitialPoint();
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "robot_controller_node");
    
    RobotController rc;

	return 0;
}