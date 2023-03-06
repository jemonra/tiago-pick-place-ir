#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <assignment_1/TiagoControllerAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <assignment_1/detection.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class TiagoControllerAction
{
	protected:
	
	// Node handle for this class
	ros::NodeHandle nh_;

	// This action server (tiago_controller) object 
	actionlib::SimpleActionServer<assignment_1::TiagoControllerAction> as_;
	
	// Action name
	std::string action_name_;
	
	// This action feedback object
	assignment_1::TiagoControllerFeedback feedback_;

	// This action result object
	assignment_1::TiagoControllerResult result_;

	// Client for the ROS server obstacle_detection (created by us)
	ros::ServiceClient clientObstacleDetection;

	// true -> after moving the robot obstacle detection will be performed
	// false -> after moving the robot obstacle detection will not be performed
	bool obstacleDetection;

	public:
	// Constructor
	TiagoControllerAction(std::string name) : as_(nh_, name, boost::bind(&TiagoControllerAction::executeCB, this, _1), false), action_name_(name)
	{
		as_.start();
		clientObstacleDetection = nh_.serviceClient<assignment_1::detection>("obstacle_detection");
	}
	
    // Destructor
	~TiagoControllerAction(void){}

	/**
	 * Send a feedback message, with a different mode, to the client.
	 *
	 * The feedback messages we have set for this action server are:
	 *	0 -> goal received
	 *	1 -> position sent to navigation stack (action move_base)
	 *	2 -> new position reached
	 *	3 -> reached final position, calling obstacle detection
	 *
	 * @param mode Mode that specifies the type of feedback to send.
	 * @return Does not return anything (void function).
	 */
	void sendFeedback(int mode)
	{
		if (mode == 0) {
			ROS_INFO("(tiago_controller) Sent feedback mode 0: Goal received");
			feedback_.mode = 0;
		} else if (mode == 1) {
			ROS_INFO("(tiago_controller) Sent feedback mode 1: Position sent to navigation stack through move_base action");
			feedback_.mode = 1;
		} else if (mode == 2) {
			ROS_INFO("(tiago_controller) Sent feedback mode 2: List of poses");
			feedback_.mode = 2;
		} else if (mode == 3) {
			ROS_INFO("(tiago_controller) Sent feedback mode 3: Reached final position, calling obstacle detection");
			feedback_.mode = 3;
		} else {
			ROS_INFO("(tiago_controller) ERROR: unknown feedback mode");
		}
		as_.publishFeedback(feedback_); // send the feedback
	}

	/**
	 * Print a laser scan message to debug
	 *
	 * Print a sensor_msgs::LaserScan message showing: angle_min, angle_min, 
	 * angle_increment, time_increment, scan_time, range_min, range_max, and all
	 * the detected points.
	 *
	 * @param msg Laser scan message.
	 * @return Does not return anything (void function).
	 */

	void printLaserScan(const sensor_msgs::LaserScan& msg)
	{
		ROS_INFO("\tangle_min: %f", msg.angle_min);
		ROS_INFO("\tangle_max: %f", msg.angle_max);
		ROS_INFO("\tangle_increment: %f", msg.angle_increment);
		ROS_INFO("\ttime_increment: %f", msg.time_increment);
		ROS_INFO("\tscan_time: %f", msg.scan_time);
		ROS_INFO("\trange_min: %f", msg.range_min);
		ROS_INFO("\trange_max: %f", msg.range_max);

		const auto& ranges = msg.ranges;
		const auto& intensities = msg.intensities;

		std::string rangesStr = "";
		for (float r : ranges)
		{
			if (!isinf(r)) // if range is not inf -> print
			{
				rangesStr += std::to_string(r) + " ";
			}
		}
		ROS_INFO("\tranges: [%s]", rangesStr.c_str());

		std::string intensitiesStr = "";
		for (float i : intensities)
		{
			if (i > 0) // if intensity is not 0 -> print
			{
				intensitiesStr += std::to_string(i) + " ";
			}
		}
		ROS_INFO("\tintensities: [%s]", intensitiesStr.c_str());
	}

	/**
	 * Done callback of the action server move_base.
	 *
	 * This function is called when the action server move_base has finished
	 * of moving the robot. It sends to us the state and the result of the
	 * movement.
	 * If the action server move_base moved the robot correctly, then the 
	 * obstacle detection server (obstacle_detection) is called.
	 * The result obtained from the obstacle_detection server is returned to the
	 * action client.
	 *
	 * @param state State of the movement returned by move_base action.
	 * @param result Result of the movement returned by move_base action.
	 * @return Does not return anything (void function). Anyway, it returns the list
	 * of obstacles to the action client by modifying the attribute result_.
	 */
	void doneMoveBase(const actionlib::SimpleClientGoalState& state,
						const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("(move_base) Received result");
		if(state == actionlib::SimpleClientGoalState::SUCCEEDED) // move_base moved the robot
		{
			ROS_INFO("The base finished moving!");
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			
			if (obstacleDetection) // If we were asked to perform obstacle detection
			{
				// Now we must do obstacle detection
				ROS_INFO("Calling server obstacle_detection");
				sendFeedback(3);
			
				assignment_1::detection srv;
				srv.request.scanMinusMapThreshold = 0.8;
				srv.request.obstacleDetectionThreshold = 0.6;
				// These thresholds have been set empirically, trying different configurations

				if(clientObstacleDetection.call(srv)) // call the obstacle detection server
				{
					// Print all obstacles detected
					for (geometry_msgs::Point obs : srv.response.obsPoint)
					{
						ROS_INFO("(obstacle_detection) Obstacle detected in (%4.2f, %4.2f)", obs.x, obs.y);
					}
				}
				else // ERROR with obstacle_detection server
				{  
					ROS_ERROR("Failed to call service obstacle_detection");
					
					result_.obstacles.clear();

					ROS_INFO("(tiago_controller) Sending result, aborted");
					as_.setAborted(result_);

					return;
				}
				
				result_.obstacles = srv.response.obsPoint;
			}

			ROS_INFO("(tiago_controller) Sending result, success");
			as_.setSucceeded(result_);
		}
		else // ERROR with move_base action server
		{
			ROS_INFO("The base failed to move!");

			result_.obstacles.clear();

			ROS_INFO("(tiago_controller) Sending result, aborted");
			as_.setAborted(result_);

			return;
		}
	}

	/**
	 * Feedback callback of the action server move_base
	 *
	 * This function is called when the action server move_base has moved the robot
	 * to a new position, and a feedback message with the new pose reached is sent.
	 * We get this pose in the feedback message and we add it to the list of poses
	 * reached by the robot in our action client feedback message (attribute feedback_).
	 * After this, the feedback is sent to the action client,
	 *
	 * @param state Feedback of the movement returned by move_base action
	 * @return Does not return anything (void function). Anyway, it sends a feedback
	 * to the action client.
	 */
	void feedbackMoveBase(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
	{
		ROS_INFO("(move_base) Received feedback");
		feedback_.poses.push_back(feedback->base_position.pose); // save pose received as feedback

		sendFeedback(2); // send feedback
	}

	/**
	 * Method executed when this action server is called
	 *
	 * It sends the goal recevided (destination position) to the move_base action server, in order 
	 * to move the robot, and waits until this move_base server finishes. Note that the result of
	 * our action server (attribute result_) is fulfilled by the function doneMoveBase, not in this 
	 * method.
	 *
	 * @param goal Goal received from action client.
	 * @return Does not return anything (void function). Anyway, it sends the result (attribute
	 * result_) to the action client.
	 */
	void executeCB(const assignment_1::TiagoControllerGoalConstPtr &goal)
	{	
		ROS_INFO("(tiago_controller) Server received goal!"); 
		sendFeedback(0);

		// Save goal.obstacleDetection to know if obstacle detection must be performed
		obstacleDetection = goal->obstacleDetection;

		// Clear the feedback vector when the action server is called
		feedback_.poses.clear();

		// Action server to move the robot, move_base. Belongs to the navigation stack.
  		MoveBaseClient actionMoveBase("move_base", true);

  		// Wait for the action server to come up
  		while(!actionMoveBase.waitForServer(ros::Duration(5.0))){
    		ROS_INFO("(move_base) Waiting for the move_base action server to come up...");
  		}

		ROS_INFO("Sending goal to move_base"); 
		sendFeedback(1);

  		actionMoveBase.sendGoal(goal->position, 
								boost::bind(&TiagoControllerAction::doneMoveBase, this, _1, _2),
                				MoveBaseClient::SimpleActiveCallback(),
                				boost::bind(&TiagoControllerAction::feedbackMoveBase, this, _1));
		
		actionMoveBase.waitForResult(); //Needed to set as_ result, if not a empty result message is returned.
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "server_node");

	TiagoControllerAction tiagoController("tiago_controller");
	ROS_INFO("Server ready, waiting for goals...");

	ros::spin();
	
	return 0;
}