#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <assignment_1/TiagoControllerAction.h>

/**
 * Print a vector of poses.
 *
 * Iterate over the vector of poses so as to print the cartesian coordinates.
 *
 * @param poses Vector containing the robot's poses.
 * @param onlyLast Print only the last pose
 * @return Does not return anything (void function).
 */
void printPoses(std::vector<geometry_msgs::Pose> poses, bool onlyLast)
{
	if (onlyLast) {
		geometry_msgs::Point position = poses.back().position;
		ROS_INFO("POSITION x = [%4.2f], y = [%4.2f], z = [%4.2f]", position.x, position.y, position.z);

		geometry_msgs::Quaternion orientation = poses.back().orientation;
		ROS_INFO("ORIENTATION x = [%4.2f], y = [%4.2f], z = [%4.2f], w = [%4.2f]", orientation.x, orientation.y, orientation.z, orientation.w);
	} else {
		for(int i = 0; i < poses.size(); i++)	
		{
			geometry_msgs::Point position = poses.at(i).position;
			ROS_INFO("POSITION x = [%4.2f], y = [%4.2f], z = [%4.2f]", position.x, position.y, position.z);

			geometry_msgs::Quaternion orientation = poses.at(i).orientation;
			ROS_INFO("ORIENTATION x = [%4.2f], y = [%4.2f], z = [%4.2f], w = [%4.2f]", orientation.x, orientation.y, orientation.z, orientation.w);
		}
	}
}

/**
 * Print the obstacles in the environment.
 *
 * It shows the cartesian coordinates of each point.
 *
 * @param obstacles Array containing the points that have been detected .
 * @return Does not return anything (void function).
 */
void printObstacles(std::vector<geometry_msgs::Point> obstacles)
{
	for(int i = 0; i < obstacles.size(); i++)	
	{
		ROS_INFO("POSITION x = [%4.2f], y = [%4.2f], z = [%4.2f]", obstacles[i].x, obstacles[i].y, obstacles[i].z);
	}
}

/**
 * Callback that gets called when the tiago_controller action server finishes its execution.
 *
 * Not only does it print the final goal's state, but also (if succeeded) the final results
 * (obstacles).
 *
 * @param state The final goal's state.
 * @param result The result sent by the server.
 * @return Does not return anything (void function).
 */
void doneCb(const actionlib::SimpleClientGoalState& state, const assignment_1::TiagoControllerResultConstPtr& result)
{
	ROS_INFO("Result received");

	if(state == actionlib::SimpleClientGoalState::ABORTED)
	{
		ROS_INFO("The goal was aborted during execution by the action server due to some failure");
	}
	else if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("The goal was achieved successfully by the action server, obstacles:");
		printObstacles(result->obstacles);
	}
	else
	{
		ROS_INFO("ERROR: Unknown state");
	}
}

/**
 * Callback that gets called on transitions to active of the tiago_controller action server.
 *
 * @return Does not return anything (void function).
 */
void activeCb()
{
  ROS_INFO("The goal is active");
}

/**
 * Callback that gets called whenever feedback for tiago_controller action server is received.
 *
 * This method prints the feedback received from the server.
 *
 * @param feedback Feedback comming from the server.
 * @return Does not return anything (void function).
 */
void feedbackCb(const assignment_1::TiagoControllerFeedbackConstPtr& feedback)
{
	ROS_INFO("Feedback received, mode: %d", feedback->mode);

	if (feedback->mode == 0) {
		ROS_INFO("Goal received by action server");
	} else if (feedback->mode == 1) {
		ROS_INFO("Position sent to navigation stack by action server");
	} else if (feedback->mode == 2) {
		ROS_INFO("List of robot poses, last received:");
		// Print last pose
		printPoses(feedback->poses, true);
	} else if (feedback->mode == 3) {
		ROS_INFO("Reached final position, calling obstacle detection");
	} else {
		ROS_ERROR("ERROR: unknown feedback mode");
	}
}

geometry_msgs::Quaternion zRotation(double theta)
{
	geometry_msgs::Quaternion q;
	q.x = 0;
	q.y = 0;
	q.z = sin(theta/2);
	q.w = cos(theta/2);
	return q;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "client_node");
	actionlib::SimpleActionClient<assignment_1::TiagoControllerAction> ac("tiago_controller", true);

	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer(); //will wait infinite time
	
	ROS_INFO("Action server started, sending goal.");

	double x, y, theta;

	// Set the goal (position and orientation)
	assignment_1::TiagoControllerGoal goal;

	goal.position.target_pose.header.frame_id = "map";
	goal.position.target_pose.header.stamp = ros::Time::now();

	goal.position.target_pose.pose.position.z = 0;

	// Check whether the user has provided the goal pose or not
	if (argc == 1) // Goal pose not provided in args
	{
		std::cout << "Insert position x: "  << "\n";
		std::cin >> x;

		std::cout << "Insert position y: "  << "\n";
		std::cin >> y;

		std::cout << "Insert rotation theta: "  << "\n";
		std::cin >> theta;
		
		goal.position.target_pose.pose.position.x = x;
		goal.position.target_pose.pose.position.y = y;

		goal.position.target_pose.pose.orientation = zRotation(theta*M_PI/180.0);
	}
	else // Goal pose provided in args
	{
		goal.position.target_pose.pose.position.x = std::stod(argv[1]);
		goal.position.target_pose.pose.position.y = std::stod(argv[2]);
	}
   
    ROS_INFO("Sending goal.");
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	
	ac.waitForResult();

	return 0;
}