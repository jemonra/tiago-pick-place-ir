#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <assignment_2/ObjectManipulationAction.h>
#include <assignment_2/ObjectWithTag.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gazebo_ros_link_attacher/Attach.h>

class ObjectManipulationAction
{
	protected:
	
	// Node handle for this class
	ros::NodeHandle nh_;

	// This action server (tiago_controller) object 
	actionlib::SimpleActionServer<assignment_2::ObjectManipulationAction> as_;
	
	// Action name
	std::string action_name_;
	
	// This action feedback object
	assignment_2::ObjectManipulationFeedback feedback_;

	// This action result object
	assignment_2::ObjectManipulationResult result_;

    // MOVE IT

    // To allow the MoveGroupInterface to get information about the robot's
    // state. One way to do this is to start an AsyncSpinner beforehand.
    ros::AsyncSpinner spinner_;

    // List of collision objects created by us
    std::vector<moveit_msgs::CollisionObject> collisionObjects_;

    // Object used to move the tiago arm ("arm_torso"). mgi stands for Move
    // Group Interface.
    moveit::planning_interface::MoveGroupInterface mgi_arm_;

    // Object used to move the gripper
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> gripperClient_;

    // Planning scene where the collision objects are added.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    // Transformation from map points to robot points
    geometry_msgs::TransformStamped transformMap2Robot;

    // GAZEBO_ROS_LINK_ATTACHER

    // Client for service /link_attacher_node/attach
    ros::ServiceClient attachService_;

    // Client for service /link_attacher_node/detach 
    ros::ServiceClient detachService_;

	public:
	// Constructor
	ObjectManipulationAction(std::string name) : as_(nh_, name, boost::bind(&ObjectManipulationAction::executeCB, this, _1), false), 
                                                 action_name_(name),
                                                 spinner_(1),
                                                 mgi_arm_("arm_torso")
	{
        gripperClient_.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/gripper_controller/follow_joint_trajectory"));
        // Wait for gripper controller action server to come up
        while(!gripperClient_->waitForServer(ros::Duration(2.0)) && ros::ok())
        {
            ROS_DEBUG("Waiting for the parallel_gripper_controller_action server to come up");
        }
        ROS_INFO("Action server for gripper is ready");

        attachService_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
        attachService_.waitForExistence();
        ROS_INFO("Service server /link_attacher_node/attach is ready");

        detachService_ = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
        detachService_.waitForExistence();
        ROS_INFO("Service server /link_attacher_node/detach is ready");

        spinner_.start();        
		as_.start();

        ROS_INFO("Action server object_manipulation ready, waiting for goals...");      
	}
	
    // Destructor
	~ObjectManipulationAction(void)
    {
        spinner_.stop();
    }
    
    /***********************
     * TF2
    ***********************/

    /**
     * Move a given pose from one reference frame to another
     *
     * Apply a tf2 transform to a pose object to get the pose with respect to
     * other reference frame
     *
     * @param pose Pose to be transformed.
     * @param transform Transformation matrix that will be applied.
     * @return Transformed pose.
     */
    geometry_msgs::Pose applyTransform(geometry_msgs::Pose pose,
                                                    const geometry_msgs::TransformStamped& transform)
    {
        geometry_msgs::Pose newPose;
        tf2::doTransform(pose, newPose, transform); // apply transformation

        return newPose;
    }

    /***********************
     * COLLISION OBJECTS
    ***********************/

    /**
     * Adds all the map immovable collision objects to the planning scene.
     *
     * Adds the map collision objects to the planning scene; these are:
     * pick_table, place_table_b, plaze_table_g, place_table_r,
     * construction_cone_1 and construction_cone_4. Their positions have been
     * obtained using RVIZ and the robot laser scanner; their dimensions have
     * been obtained using Gazebo and the .world models. As they don't change,
     * all the data is written here, in this function. They are added to the
     * collisionObjects_ attribute, but also to the robot's planning scene
     *
     * @return void.
    */
    void addImmovableCollisionObjects()
    {        
        // List of immovable object data pick_table
        moveit_msgs::CollisionObject pickTable;

        pickTable.id = "pick_table";
        pickTable.header.frame_id = "map";

        pickTable.primitives.resize(1);
        pickTable.primitives[0].type = pickTable.primitives[0].BOX;
        pickTable.primitives[0].dimensions.resize(3);
        pickTable.primitives[0].dimensions[0] = 0.91 + 0.1; // x + error
        pickTable.primitives[0].dimensions[1] = 0.92 + 0.1; // y + error
        pickTable.primitives[0].dimensions[2] = 0.775; // z

        pickTable.primitive_poses.resize(1);
        pickTable.primitive_poses[0].position.x = 7.8;
        pickTable.primitive_poses[0].position.y = -2.95;
        pickTable.primitive_poses[0].position.z = 0.3875;

        pickTable.operation = pickTable.ADD;

        collisionObjects_.push_back(pickTable);

        // place_table_b
        moveit_msgs::CollisionObject placeTableB;
        placeTableB.id = "place_table_b";
        placeTableB.header.frame_id = "map";

        placeTableB.primitives.resize(1);
        placeTableB.primitives[0].type = placeTableB.primitives[0].CYLINDER;
        placeTableB.primitives[0].dimensions.resize(2);
        placeTableB.primitives[0].dimensions[0] = 0.69; // Height
        placeTableB.primitives[0].dimensions[1] = 0.21; // Radious

        placeTableB.primitive_poses.resize(1);
        placeTableB.primitive_poses[0].position.x = 12.52;
        placeTableB.primitive_poses[0].position.y = -0.35;
        placeTableB.primitive_poses[0].position.z = 0.345000;

        placeTableB.operation = placeTableB.ADD;

        collisionObjects_.push_back(placeTableB);

        // place_table_g
        moveit_msgs::CollisionObject placeTableG;
        placeTableG.id = "place_table_g";
        placeTableG.header.frame_id = "map";

        placeTableG.primitives.resize(1);
        placeTableG.primitives[0].type = placeTableG.primitives[0].CYLINDER;
        placeTableG.primitives[0].dimensions.resize(2);
        placeTableG.primitives[0].dimensions[0] = 0.69; // Height
        placeTableG.primitives[0].dimensions[1] = 0.21; // Radious

        placeTableG.primitive_poses.resize(1);
        placeTableG.primitive_poses[0].position.x = 11.52;
        placeTableG.primitive_poses[0].position.y = -0.35;
        placeTableG.primitive_poses[0].position.z = 0.345000;

        placeTableG.operation = placeTableG.ADD;

        collisionObjects_.push_back(placeTableG);

        // place_table_r
        moveit_msgs::CollisionObject placeTableR;
        placeTableR.id = "place_table_r";
        placeTableR.header.frame_id = "map";

        placeTableR.primitives.resize(1);
        placeTableR.primitives[0].type = placeTableR.primitives[0].CYLINDER;
        placeTableR.primitives[0].dimensions.resize(2);
        placeTableR.primitives[0].dimensions[0] = 0.69; // Height
        placeTableR.primitives[0].dimensions[1] = 0.21; // Radious

        placeTableR.primitive_poses.resize(1);
        placeTableR.primitive_poses[0].position.x = 10.52;
        placeTableR.primitive_poses[0].position.y = -0.34;
        placeTableR.primitive_poses[0].position.z = 0.345000;
        
        placeTableR.operation = placeTableR.ADD;

        collisionObjects_.push_back(placeTableR);

        // construction_cone_1
        moveit_msgs::CollisionObject constructionCone1;
        constructionCone1.id = "construction_cone_1";
        constructionCone1.header.frame_id = "map";

        constructionCone1.primitives.resize(1);
        constructionCone1.primitives[0].type = constructionCone1.primitives[0].CYLINDER;
        constructionCone1.primitives[0].dimensions.resize(2);
        constructionCone1.primitives[0].dimensions[0] = 1.457100; // Height
        constructionCone1.primitives[0].dimensions[1] = 0.183096; // Radious

        constructionCone1.primitive_poses.resize(1);
        constructionCone1.primitive_poses[0].position.x = 11.725;
        constructionCone1.primitive_poses[0].position.y = -3.07;
        constructionCone1.primitive_poses[0].position.z = 0.728548;

        constructionCone1.operation = constructionCone1.ADD;

        collisionObjects_.push_back(constructionCone1);

        // construction_cone_4
        moveit_msgs::CollisionObject constructionCone4;
        constructionCone4.id = "construction_cone_4";
        constructionCone4.header.frame_id = "map";

        constructionCone4.primitives.resize(1);
        constructionCone4.primitives[0].type = constructionCone4.primitives[0].CYLINDER;
        constructionCone4.primitives[0].dimensions.resize(2);
        constructionCone4.primitives[0].dimensions[0] = 1.457100; // height
        constructionCone4.primitives[0].dimensions[1] = 0.183096 + 0.01; // radious + error

        constructionCone4.primitive_poses.resize(1);
        constructionCone4.primitive_poses[0].position.x = 7.35;
        constructionCone4.primitive_poses[0].position.y = -0.72;
        constructionCone4.primitive_poses[0].position.z = 0.728548;

        constructionCone4.operation = constructionCone4.ADD;

        collisionObjects_.push_back(constructionCone4);

        planning_scene_interface_.applyCollisionObjects(collisionObjects_); // Add collision objects to planing scene
    }

    /**
     * Adds all the detected collision objects to the planning scene.
     *
     * Adds all the objects detected in the camera to the planning scene. The
     * objects detected and their poses must be passed as argument. This
     * function adds their dimensions (got using Gazemo and .world files) and
     * includes them to the planning scene. Note that these objects are not
     * fixed in the environment like the previous ones.
     *
     * @param objects Vector of objects detected by the camera to add to the
     * planning scene.
     *
     * @return void.
    */
    void addMovableCollisionObjects(std::vector<assignment_2::ObjectWithTag> objects)
    {
        for (assignment_2::ObjectWithTag o : objects)
        {
            moveit_msgs::CollisionObject collisionObject;
            collisionObject.id = "movable"+std::to_string(o.id);
            collisionObject.header.frame_id = "base_footprint";

            collisionObject.primitives.resize(1);
            
            if (o.id == 1) //blue
            {
                collisionObject.primitives[0].type = collisionObject.primitives[0].CYLINDER;
                collisionObject.primitives[0].dimensions.resize(2);

                collisionObject.primitives[0].dimensions[0] = 0.1; // Height
                collisionObject.primitives[0].dimensions[1] = 0.03; // Radious
            }
            else if (o.id == 2) //green_triangle
            {
                collisionObject.primitives[0].type = collisionObject.primitives[0].BOX;
                collisionObject.primitives[0].dimensions.resize(3);

                collisionObject.primitives[0].dimensions[0] = 0.07; // + error
                collisionObject.primitives[0].dimensions[1] = 0.05; // + error
                collisionObject.primitives[0].dimensions[2] = 0.05; // + error
            }
            else if (o.id == 3) //red_cube
            {
                collisionObject.primitives[0].type = collisionObject.primitives[0].BOX;
                collisionObject.primitives[0].dimensions.resize(3);

                collisionObject.primitives[0].dimensions[0] = 0.05;
                collisionObject.primitives[0].dimensions[1] = 0.05;
                collisionObject.primitives[0].dimensions[2] = 0.05;
            }
            else if (o.id == 4 || o.id == 5 || o.id == 6 || o.id == 7) //gold_obs in table
            {
                collisionObject.primitives[0].type = collisionObject.primitives[0].CYLINDER;
                collisionObject.primitives[0].dimensions.resize(2);

                collisionObject.primitives[0].dimensions[0] = 0.2; // Height
                collisionObject.primitives[0].dimensions[1] = 0.05; // Radious
            }

            collisionObject.primitive_poses.resize(1);

            collisionObject.primitive_poses[0] = o.pose;

            // TEMPORARY
            collisionObject.primitive_poses[0].position.z -= collisionObject.primitives[0].dimensions[0]/2;

            collisionObject.operation = collisionObject.ADD;

            collisionObjects_.push_back(collisionObject);
        }

        planning_scene_interface_.applyCollisionObjects(collisionObjects_); // Add collision objects to planing scene
    }

    /**
     * Removes all the collision objects from the planning scene.
     *
     * Iterates over the 'collisionObjects_' list to remove all of them from the
     * planning scene. This function should be called at the end of this action
     * callback.
     *
     * @return void.
    */
    void removeAllCollisionObjects()
    {
        std::vector<std::string> object_ids;
        for (moveit_msgs::CollisionObject co : collisionObjects_)
        {
            object_ids.push_back(co.id);
        }
        planning_scene_interface_.removeCollisionObjects(object_ids);
    }

    /**
     * Removes a collision objects of from the planning scene.
     *
     * Searches in the collisionObjects_ vector a collision object with id =
     * "movable"+id; then: (1) removes this collision object from the planning
     * scene (2) removes this collision object from the collisionObjects_
     * vector.
     *
     * @param id Id of the (movable) collision object to remove.
     *
     * @return true if the collision object was removed successfully; false
     * otherwise.
    */
    bool removeCollisionObject(int id)
    {
        moveit_msgs::CollisionObject coRemove;
        int i = 0;
        bool found = false;
        while(i < collisionObjects_.size() && !found)
        {
            coRemove = collisionObjects_.at(i);
            if (coRemove.id == "movable"+std::to_string(id))
            {
                found = true;
            }
            i++;
        }
        if (!found)
        {
            ROS_ERROR("Collision object with id=%d not found in list", id);
            return false;
        }

        std::vector<std::string> removeIds; 
        removeIds.push_back(coRemove.id); 
        planning_scene_interface_.removeCollisionObjects(removeIds);
    }

    /***********************
     * ARM
    ***********************/

    /**
     * Returns a quaternion corresponding to the new gripper orientation
     *
     * Defines the orientation of the gripper based on Euler angles.
     *
     * @param roll Euler angle along the x axis.
     * @param pitch Euler angle along the y axis.
     * @param yaw Euler angle along the z axis.
     *
     * @return Quaternion with the orientation for a gripper
    */
    geometry_msgs::Quaternion gripperLookingAt(double roll, double pitch, double yaw)
    {
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(roll, pitch, yaw);
        myQuaternion = myQuaternion.normalize();
        return tf2::toMsg(myQuaternion);
    }

    /**
     * Returns a quaternion corresponding to the gripper orientation looking at
     * the floor.
     *
     * This gripper orientation will be needed when catching objects vertically
     * (approachArmAbove function).
     *
     * @return Quaternion with the orientation for a gripper looking at the
     * floor.
    */
    geometry_msgs::Quaternion gripperLookingFloor()
    {
        return gripperLookingAt(0, M_PI_2, 0);
    }

    /**
     * Returns a quaternion corresponding to the gripper orientation looking at
     * the right of the robot (the camera looks forward).
     *
     * This gripper orientation will be needed when moving the arm to the secure
     * position (moveArmToSecure).
     *
     * @return Quaternion with the orientation for a gripper looking at the
     * right (robot's camera looks forward).
    */
   geometry_msgs::Quaternion gripperLookingRight()
   {
        return gripperLookingAt(0, 0, 4.71);
   }

    /**
     * Moves the arm to a position with a gripper orientation and waits until
     * the movement is finished.
     *
     * Moves the robot's arm to a position, specifyng also the gripper
     * orientation in this position. This is the basic function for moving the
     * robot's arm, all the others will be coded based on this. Waits until the
     * arm finishes the movement and returns its result. Note that the position
     * provided is with respect to the frame "base_footprint", the base of the
     * robot.
     *
     * @param position Goal position for the arm.
     * @param orientation Goal orientation for the gripper, in the position
     * specified.
     *
     * @return true if the movement was correctly completed; false otherwise.
    */
    bool moveArm(geometry_msgs::Point position, geometry_msgs::Quaternion orientation)
    {
        ROS_INFO("Moving arm to (%4.2f, %4.2f, %4.2f), (%4.2f, %4.2f, %4.2f, %4.2f)...", 
            position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w);
        
        geometry_msgs::PoseStamped goal_pose;

        // Frame: frame of the robot base
        goal_pose.header.frame_id = "base_footprint";

        // Position: provided in arguments
        goal_pose.pose.position = position;

        // Orientation: provided in arguments
        goal_pose.pose.orientation = orientation;

        // Set target
        mgi_arm_.setPoseTarget(goal_pose);

        mgi_arm_.setStartStateToCurrentState();
        mgi_arm_.setMaxVelocityScalingFactor(1.0);
        mgi_arm_.setPlanningTime(5.0);

        // Generate plan
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (mgi_arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            ROS_INFO_STREAM("\tPlan found in " << my_plan.planning_time_ << " seconds");
        } else 
        {
            ROS_ERROR("\tERROR, impossible to get a plan");
            return false;
        }

        // Execute the plan
        ros::Time start = ros::Time::now();
        mgi_arm_.move();
        ROS_INFO_STREAM("\tSuccessfully moved, motion duration: " << (ros::Time::now() - start).toSec());
        return true;
    }

    /**
     * Moves the arm to a position with a gripper orientation and waits until
     * the movement is finished.
     *
     * It is the same as the moveArm function, but the position is provided with
     * 3 double values. The position is always w.r.t. the frame
     * "base_footprint".
     *
     * @param x Coordinate x of the arm goal position.
     * @param y Coordinate y of the arm goal position.
     * @param z Coordinate z of the arm goal position.
     * @param orientation Goal orientation for the gripper, in the position
     * specified.
     *
     * @return true if the movement was correctly completed; false otherwise.
    */
    bool moveArm(double x, double y, double z, geometry_msgs::Quaternion orientation)
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        moveArm(p, orientation);
    }

    /**
     * Moves the arm some centimeters above a position specified.
     *
     * Uses the moveArm function to move the arm centimeters above the position
     * specified. This function is useful when cathing objects in the table
     * vertically. To approach the arm above a position we sum the distance to
     * the position z-axis.
     *
     * @param position Goal position for the arm, minus distance in z-axis.
     * @param distance Distance above the position specified where the arm will
     * be moved.
     *
     * @return true if the movement was correctly completed; false otherwise.
    */
    bool approachArmAbove(geometry_msgs::Point position, double distance)
    {
        position.z = position.z + distance;
        return moveArm(position, gripperLookingFloor());
    }

    /**
     * Moves the arm to a secure pose where it won't collide with the walls or
     * other objects.
     *
     * This function moves the arm to a secure pose. This pose has been set in
     * the above high part of the robot, without interfering with the camera.
     *
     * @return true if the movement was correctly completed; false otherwise.
    */
    bool moveArmToSecure()
    {
        geometry_msgs::Point position;
        position.x = 0.10;
        position.y = 0.10;
        position.z = 1.60;
        geometry_msgs::Quaternion orientation = gripperLookingRight();

        return moveArm(position, orientation);
    }

    /***********************
     * GRIPPER
    ***********************/

    /**
     * Generate a simple trajectory to move TIAGo's gripper. Simply define the
     * goal's parameters.
     *
     * @param goal Goal to be filled.
     * @param first_finger_separation Separation of the first finger of the
     * robot w.r.t. the center of the gripper.
     * @param second_finger_separation Separation of the second finger of the
     * robot w.r.t. the center of the gripper.
     *
     * @return void.
    */
    void waypoints_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal, double first_finger_separation, double second_finger_separation)
    {
        // The joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
        goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
    
        // One waypoint in this goal trajectory
        goal.trajectory.points.resize(1);
    
        // First trajectory point Positions
        int index = 0;
        goal.trajectory.points[index].positions.resize(2);
        goal.trajectory.points[index].positions[0] = first_finger_separation;
        goal.trajectory.points[index].positions[1] = second_finger_separation;
        
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
     * Moves the gripper finger to a certain position and waits until they reach
     * the positions.
     *
     * Moves both fingers of the gripper with respect to the center of it to
     * perform the operations of catching/dropping of objects.
     *
     * @param first_finger_separation Separation of the first finger of the
     * robot w.r.t. the center of the gripper.
     * @param second_finger_separation Separation of the second finger of the
     * robot w.r.t. the center of the gripper.
     *
     * @return true if the movement was correctly completed; false otherwise.
    */
    bool manipulateGripper(double first_finger_separation, double second_finger_separation)
    {
        ROS_INFO("Moving gripper to (%4.2f, %4.2f)...", first_finger_separation, second_finger_separation);

        // Generates the goal for the TIAGo's head
        control_msgs::FollowJointTrajectoryGoal gripper_goal;
        waypoints_gripper_goal(gripper_goal, first_finger_separation, second_finger_separation);

        // Sends the command to start the given trajectory 1s from now
        gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        gripperClient_->sendGoalAndWait(gripper_goal);

        if (gripperClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("\tSuccessfully moved gripper");
            return true;
        } else {
            ROS_WARN("\tRobot couldn't move gripper (it could be caught object)");
            return false;
        }
    }

    /**
     * Opens the robot's gripper.
     *
     * @return true if the movement was correctly completed; false otherwise.
    */
    bool openGripper()
    {
        double first_finger_separation_open = 0.04;
        double second_finger_separation_open = 0.04;

        return manipulateGripper(first_finger_separation_open, second_finger_separation_open);
    }

    /**
     * Closes the robot's gripper.
     *
     * @return true if the movement was correctly completed; false otherwise.
    */
    bool closeGripper()
    {
        double first_finger_separation_close = 0;
        double second_finger_separation_close = 0;

        return manipulateGripper(first_finger_separation_close, second_finger_separation_close);
    }

    /***********************
     * ATTACHING/DETACHING OBJECTS
    ***********************/

    /**
     * Attaches or detaches an object to the robot arm using the
     * gazebo_ros_link_attacher plugin.
     *
     * Sometimes catching the object is very difficult due to the oscilations
     * gripper and the control of frictions in the Gazebo simulation. In this
     * way, the two objects specified (the gripper and a movable object) remain
     * united, and if ones moves the other too. The communicacion with the
     * plugin is performed with a ROS service. Depending on the id passed to
     * this function, the ROS service will be called with the model name and
     * link name corresponding to this object.
     *
     * @param attach True if attaching action; false if detaching action. 
     * @param id Id of the movable object to attach to the robot's gripper.
     *
     * @return true if the operation was successfully performed; false
     * otherwise.
    */
    bool attachDetachObject(bool attach, int id)
    {
        gazebo_ros_link_attacher::Attach attachReqRes;
        attachReqRes.request.model_name_1 = "tiago";
        attachReqRes.request.link_name_1 = "arm_7_link";
        if (id == 1) // blue_hexagon
        {
            attachReqRes.request.model_name_2 = "Hexagon";
            attachReqRes.request.link_name_2 = "Hexagon_link";
        } else if (id == 2) // green_triangle
        {
            attachReqRes.request.model_name_2 = "Triangle";
            attachReqRes.request.link_name_2 = "Triangle_link";
        } else if (id == 3) // red_cube
        {
            attachReqRes.request.model_name_2 = "cube";
            attachReqRes.request.link_name_2 = "cube_link";
        } else {
            ROS_ERROR("\tCannot attach object with id=%d", id);
            return false;
        }

        if (attach) // ATTACH MODE
        {
            if (attachService_.call(attachReqRes))
            {
                bool ok = attachReqRes.response.ok;
                ROS_INFO("\tSuccesfully called, response: %d", ok);
                return ok;
            } 
            else
            {
                ROS_ERROR("\tFailed to call attach service");
                return false;
            }
        } else { // DETACH MODE
            if (detachService_.call(attachReqRes))
            {
                bool ok = attachReqRes.response.ok;
                ROS_INFO("\tSuccesfully called, response: %d", ok);
                return ok;
            } 
            else
            {
                ROS_ERROR("\tFailed to call detach service");
                return false;
            }
        }        
    }

    /**
     * Attaches an object to the robot arm using the gazebo_ros_link_attacher
     * plugin.
     *
     * This function simply calls attachDetachObject with attach = true.
     *
     * @param id Id of the movable object to attach to the robot's gripper.
     *
     * @return true if the operation was successfully performed; false
     * otherwise.
    */
    bool attachObject(int id)
    {
        ROS_INFO("Attaching object with id=%d...", id);
        return attachDetachObject(true, id);
    }

    /**
     * Detaches an object to the robot arm using the gazebo_ros_link_attacher
     * plugin.
     *
     * This function simply calls attachDetachObject with attach = false.
     *
     * @param id Id of the movable object to attach to the robot's gripper.
     *
     * @return true if the operation was successfully performed; false
     * otherwise.
    */
    bool detachObject(int id)
    {
        ROS_INFO("Detaching object with id=%d...", id);
        return attachDetachObject(false, id);
    }

    /***********************
     * OTHERS
    ***********************/

    /**
     * Explain the goal received by this action server
     *
     * This function explain the attributes 'mode', 'ids', 'poses' and
     * 'catch_id' of the action object_manipulation goal. Debug function.
     *
     * @param goal Goal to be explained (Const Ptr)
     * @return void
    */
    void explainGoal(const assignment_2::ObjectManipulationGoalConstPtr &goal)
    {
        ROS_INFO("Explaining goal received...");
        if (goal->mode == 0) // Catch object
        {
            ROS_INFO("\tMode: CATCH object with id=%d", goal->catch_id);

            ROS_INFO("\tObjects received: ");
            for (assignment_2::ObjectWithTag o : goal->objects)
            {
                ROS_INFO("\tO(id=%d; pose:%4.2f, %4.2f, %4.2f)",
                            (int)o.id,
                            o.pose.position.x,
                            o.pose.position.y,
                            o.pose.position.z);
            }
        }
        else if (goal->mode == 1) // Drop object
        {
            ROS_INFO("\tMode: DROP object");
        }
    }

    /**
     * Finds an object based on its id and returns a copy of it.
     *
     * Iterates over the vector passed as argument searching for the object with
     * id=catch_id. When it is found, it returns a copy. If it is not found
     * prints a ROS_ERROR but returns an empty object.
     *
     * @param objects Vector of objects to search.
     * @param catch_id Id of the object that must be found.
     *
     * @return A copy of the object found if found; an empty object if not
     * found.
    */
    assignment_2::ObjectWithTag findObjectWithId(std::vector<assignment_2::ObjectWithTag> objects, int catch_id)
    {
        assignment_2::ObjectWithTag resultObject;

        bool catchIdFound = false;
        int idx = 0;
        while(!catchIdFound)
        {
                assignment_2::ObjectWithTag object = objects.at(idx);
                if(object.id == catch_id)
                {
                    resultObject = object;
                    catchIdFound = true;
                }

                idx++;
        }
        
        if (!catchIdFound)
        {
            ROS_ERROR("ERROR, id=%d not found in vector of objects", catch_id);
        }

        return resultObject;
    }
    
    /**
     * This action server callback, executed whenever is called.
     *
     * This function will be execute every time this action server is called,
     * this is, it receives a goal. It adds the immovable collision objects to
     * the planning scene and depending on the goal->mode performs some
     * operations. If goal->mode == 0 (CATHING ACTION) it adds the movable
     * collision objects to the planning scene and moves the arm and the gripper
     * in order to catch the object. If goal->mode == 1 (DROPPING ACTION) it
     * moves the arm and the gripper properly in order to achieve the goal. Note
     * that, in both cases, it will leave the arm, and remove the collision
     * objects at the end of the execution.
     *
     * @param goal Goal received from action client.
     *
     * @return void. Anyway it returns a result to the action client.
    */
    void executeCB(const assignment_2::ObjectManipulationGoalConstPtr &goal)
    {
        ROS_INFO("Server received goal!");
        explainGoal(goal);

        //std::vector<assignment_2::ObjectWithTag> objectsReceived =
        //greenTriangleCorrection(goal->objects);
        std::vector<assignment_2::ObjectWithTag> objectsReceived = goal->objects;

        // Add all collision objects (immovable and movable) to planning scene
        collisionObjects_.clear();
        addImmovableCollisionObjects(); // Add immovable collision objects to vector
        ROS_INFO("Added immovable collision objects to the planning scene");
        
        if (goal->mode == 0) //CATCH mode
        {
            addMovableCollisionObjects(objectsReceived); // Add movible collision objects to vector
            ROS_INFO("Added movable collision objects to the planning scene");

            geometry_msgs::Pose object_pose; // pose of the object we want to pick up
            object_pose = findObjectWithId(objectsReceived, goal->catch_id).pose;

            openGripper();
            
            double catch_distance; // Distance to the object when catching it.
            double target_distance; // Distance to the object in the target position.

            // Specify target_distance and catch_distance depending on the
            // object to catch.
            if (goal->catch_id == 1) // blue hexagon
            {
                target_distance = 0.31;
                catch_distance = 0.20;
            }
            else if (goal->catch_id == 2) // green triangle
            {
                target_distance = 0.31;
                catch_distance = 0.24;
            }
            else if (goal->catch_id == 3) // red cube
            {
                target_distance = 0.25;
                catch_distance = 0.20;
            } else
            {
                ROS_ERROR("Not found catch_id=%d", goal->catch_id);
            }

            approachArmAbove(object_pose.position, target_distance); // (5) target position

            approachArmAbove(object_pose.position, catch_distance); // (6) object position

            //ros::Duration(240).sleep(); 

            removeCollisionObject(goal->catch_id); // (7)

            attachObject(goal->catch_id); // (8) attach virtually the object to the gripper

            closeGripper(); // (9)

            approachArmAbove(object_pose.position, target_distance); // (10) target position

            moveArmToSecure(); // (12)
        } 
        else if (goal->mode == 1)
        {
            moveArm(0.8, 0, 1, gripperLookingFloor());

            openGripper();

            detachObject(goal->catch_id);

            moveArmToSecure();
        }
        
        removeAllCollisionObjects();

        as_.setSucceeded(result_);
    }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "object_manipulation_node");

	ObjectManipulationAction tiagoController("object_manipulation");

	ros::waitForShutdown(); // instead of ros::spin()
	
	return 0;
}