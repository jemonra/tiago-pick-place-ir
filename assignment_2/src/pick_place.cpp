#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

    /*void waypoints_gripper_goal(control_msgs::FollowJointTrajectoryGoal& goal, double first_finger_separation, double second_finger_separation)
    {
        // The joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
        goal.trajectory.joint_names.push_back("gripper_right_finger_joint");
    
        // One waypoint in this goal trajectory
        goal.trajectory.points.resize(1);
    
        // First trajectory point
        // Positions
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

    void manipulateGripper(double first_finger_separation, double second_finger_separation)
    {
        ROS_INFO("Method 'openGripper' executed");

        boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> GripperClient; // Pointer

        ROS_INFO("Creating action client to gripper controller ...");
        GripperClient.reset(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/gripper_controller/follow_joint_trajectory"));

        // Wait for arm controller action server to come up
        while(!GripperClient->waitForServer(ros::Duration(2.0)) && ros::ok())
        {
            ROS_DEBUG("Waiting for the parallel_gripper_controller_action server to come up");
        }

        ROS_INFO("Method 'openGripper' is being executed");

        // Generates the goal for the TIAGo's head
        control_msgs::FollowJointTrajectoryGoal gripper_goal;
        waypoints_gripper_goal(gripper_goal, first_finger_separation, second_finger_separation);

        // Sends the command to start the given trajectory 1s from now
        gripper_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        GripperClient->sendGoalAndWait(gripper_goal);
    }*/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_place_node");
    ros::NodeHandle n;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    static const std::string PLANNING_GROUP_ARM = "arm_torso";
    moveit::planning_interface::MoveGroupInterface mgi_arm(PLANNING_GROUP_ARM);

    mgi_arm.setPlannerId("SBLkConfigDefault");
    mgi_arm.setPoseReferenceFrame("base_footprint");

    // Moving example
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.frame_id = "base_footprint";

    std::cout << "Insert position x: "  << "\n";
	std::cin >> goal_pose.pose.position.x;

	std::cout << "Insert position y: "  << "\n";
	std::cin >> goal_pose.pose.position.y;

	std::cout << "Insert position z: "  << "\n";
	std::cin >> goal_pose.pose.position.z;
    
    tf2::Quaternion myQuaternion;
    double r, p, y;

    std::cout << "Insert orientation raw: "  << "\n";
	std::cin >> r;

    std::cout << "Insert orientation pitch: "  << "\n";
	std::cin >> p;

    std::cout << "Insert orientation yaw: "  << "\n";
	std::cin >> y;

    myQuaternion.setRPY(r, p, y);
    myQuaternion = myQuaternion.normalize();
    goal_pose.pose.orientation = tf2::toMsg(myQuaternion);

    mgi_arm.setPoseTarget(goal_pose);

    ROS_INFO_STREAM("Planning to move " <<
        mgi_arm.getEndEffectorLink() << " to a target pose expressed in " <<
        mgi_arm.getPlanningFrame());

    mgi_arm.setStartStateToCurrentState();
    mgi_arm.setMaxVelocityScalingFactor(1.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    mgi_arm.setPlanningTime(5.0);

    moveit::core::MoveItErrorCode error = mgi_arm.plan(my_plan);
    bool success = (mgi_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        ROS_INFO_STREAM("\tPlan found in " << my_plan.planning_time_ << " seconds");
    } else 
    {
        ROS_ERROR("\tERROR, impossible to get a plan");
    }

    // Execute the plan
    ros::Time start = ros::Time::now();

    mgi_arm.move();

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    spinner.stop();
}