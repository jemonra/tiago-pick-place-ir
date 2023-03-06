#include <ros/ros.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <assignment_2/object_detections.h>
#include <assignment_2/ObjectWithTag.h>

class ObjectDetection
{
    protected:
    
    // Node handle for this class
	ros::NodeHandle nh_;
    
    // Service to provide the robot with information about the objects
    // (This service)
    ros::ServiceServer service_;

    // List of objects with tags (id, pose)
    std::vector<assignment_2::ObjectWithTag> objects_;

    // Transform from camera frame to robot frame
    geometry_msgs::TransformStamped transformCamera2Robot_;

    public:

    //Constructor
    ObjectDetection()
    {
        objects_.clear();

        // Start this service
        service_ = nh_.advertiseService("object_detections", &ObjectDetection::executeCB, this);

        ROS_INFO("Service server object_detections ready, waiting for requests...");
    }

    /**
     * Transform a given pose from one reference frame to another.
     *
     * Apply a tf2 transform to a pose object to get the pose with respect 
     * to another reference frame.
     *
     * @param pose Pose to be transformed.
     * @param transform Transformation matrix that will be applied.
     * 
     * @return Transformed pose.
     */
    geometry_msgs::Pose applyTransform(geometry_msgs::Pose pose, const geometry_msgs::TransformStamped& transform)
    {
        geometry_msgs::Pose newPose;
        tf2::doTransform(pose, newPose, transform); // apply transformation
        return newPose;
    }

    /**
     * Transform the pose of all objects in a vector from one reference 
     * frame to another.
     * 
     * Apply a tf2 transform to every object pose, in this way the
     * objects are represented with respect to another reference frame.
     * 
     * @param objects List of objects whose pose will be transformed.
     * @param transform Transformation matrix that will be applied.
     * 
     * @return List of objects with transformed poses.
    */
    std::vector<assignment_2::ObjectWithTag> applyTransformObjects( std::vector<assignment_2::ObjectWithTag> objects, 
                                                        const geometry_msgs::TransformStamped& transform)
    {
        std::vector<assignment_2::ObjectWithTag> result;
        for (assignment_2::ObjectWithTag o : objects) // for every object
        {
            assignment_2::ObjectWithTag newO;
            newO.id = o.id;
            newO.pose = applyTransform(o.pose, transform);
            result.push_back(newO);
        }
        return result;
    }
                                                            
    /**
     * Transform a tag_detections topic message to a vector of objects.
     * 
     * Iterate over a tag_detections message to get a list of detected
     * objects (specifying their id and their pose).
     * 
     * @param msg Message received from tag_detections topic.
     * 
     * @return List of objects detected in the message.
    */
    std::vector<assignment_2::ObjectWithTag> detections2objects(const apriltag_ros::AprilTagDetectionArray& msg)
    {
        std::vector<assignment_2::ObjectWithTag> result;

        const auto& array = msg.detections;
        
        // There will be only one AprilTagDetection per tag detected as we have not declared tag bundles.
        for(apriltag_ros::AprilTagDetection detection : array)
        {
            assignment_2::ObjectWithTag o; // object to be saved
            o.id = detection.id.at(0); // set object.id

            geometry_msgs::PoseWithCovarianceStamped poseWCS = detection.pose;
            geometry_msgs::PoseWithCovariance poseWithCovariance = poseWCS.pose;
            geometry_msgs::Pose pose = poseWithCovariance.pose;

            o.pose = pose; // set object.pose
            result.push_back(o);
        }
        return result;
    }

    /**
     * Function executed when this service is called.
     * 
     * It returns the tag ids and poses to the service client.
     * 
     * @param req Request received by service client.
     * @param res Response sent to service client.
     * 
     * @return true if all went correctly; false otherwise. Anyway, it returns the 
     * request with the detected objects to the service client.
    */
    bool executeCB(assignment_2::object_detections::Request  &req, assignment_2::object_detections::Response  &res)
    {
        ROS_INFO("Request received!");

        // Get ONLY 1 message from the topic tag_detections
        boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> spApriltagDetections;
        apriltag_ros::AprilTagDetectionArray apriltagDetections;
        spApriltagDetections = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections");
        if(spApriltagDetections != NULL)
        {
            apriltagDetections = *spApriltagDetections;
        }
        // The object apriltagDetections has all the detections
        ROS_INFO("\tGot 1 message from tag_detections");
        
        // Create the list of objects ObjectWithTag
        objects_ = detections2objects(apriltagDetections);
        ROS_INFO("\tCreated %d objects from detections", (int)objects_.size());

        // Get transform camera2Robot
        tf2_ros::Buffer tfBuffer;
	    tf2_ros::TransformListener tfListener(tfBuffer);

        // NOTE:
        // Tags are detected w.r.t. "xtion_rgb_optical_frame" frame.
            // Discovered: rostopic echo /xtion/rgb/image_rect_color
        // Torso arm of robot moves w.r.t. "base_footprint" frame
            // Discovered in online tutorials
        try{
            transformCamera2Robot_ = tfBuffer.lookupTransform(
                    "base_footprint", //target frame
                    "xtion_rgb_optical_frame", //source frame
                    ros::Time(0),
                    ros::Duration(5.0));
        } catch (tf2::TransformException &ex) {
            ROS_ERROR("\tCould NOT find transform: %s", ex.what());
            exit(EXIT_FAILURE);
        }
        ROS_INFO("\tGot transform camera to robot...");

        // Apply the transformation to all detected objects
        objects_ = applyTransformObjects(objects_, transformCamera2Robot_);

        ROS_INFO("\tProviding %d detected objects to the client", (int)objects_.size());
        res.objects = objects_;

        return true;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "object_detections_node");
    
    ObjectDetection ob;

    ros::spin();

	return 0;
}