#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <assignment_1/detection.h>

/**
 * Print a vector of points.
 *
 * Iterate over the vector of points so as to print the cartesian coordinates.
 *
 * @param points Vector containing the different detected points.
 * @param num Number of points to print.
 * @return Does not return anything (void function).
 */
void printPoints(std::vector<geometry_msgs::Point> points, int num)
{
    ROS_INFO("\tPrinting %d of %d points:", num, static_cast<int>(points.size()));
    int i = 0;
    for (geometry_msgs::Point p : points)
    {
        if (i > num)
            break;
        ROS_INFO("\tPOINT: (%4.2f, %4.2f, %4.2f)", p.x, p.y, p.z);
        i++;
    }
}

/**
 * Get the points detected by the LIDAR.
 *
 * Get the results provided by the LaserScan object which undergo some transformations
 * to get the cartesian coordinates. They are finally stored in a vector.
 *
 * @param msg Points detected by the LaserScan object.
 * @return A vector of points (cartesian coordinates).
 */
std::vector<geometry_msgs::Point> getScanPoints(const sensor_msgs::LaserScan& msg)
{
    const auto& ranges = msg.ranges;
    const auto& intensities = msg.intensities;

    //Get all polar coordinates (distance, angle)
    std::vector <float> pol_distances;
    std::vector <float> pol_angles;
    for (int i = 0; i < ranges.size(); i++)
    {
        float r = ranges.at(i);
        if (!isinf(r)) // if range is not inf
        {
            pol_distances.push_back(r);
            pol_angles.push_back(msg.angle_min + (double)i * msg.angle_increment);
        }
    }

    // Result vector
    std::vector<geometry_msgs::Point> points;

    double x;
    double y;

    // Get all cartesian coordinates (x, y)
    for (int i = 0; i < pol_distances.size(); i++)
    {
        x = pol_distances.at(i) * cos(pol_angles.at(i));
        y = pol_distances.at(i) * sin(pol_angles.at(i));

        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = 0;

        points.push_back(point);
    }

    return points;
}

/**
 * Get the points that belong to the map itself.
 *
 * Iterate over the occupancy grid and get the cartesian coordinates of all those cells
 * that are likely to be occupied.
 *
 * @param msg Map's occupancy grid.
 * @return Vector containing the map points.
 */
std::vector<geometry_msgs::Point> getMapPoints(const nav_msgs::OccupancyGrid& msg)
{
    std::vector<geometry_msgs::Point> points;
    
    // First of all, let's go through the map
    for (int i = 0; i <= msg.info.width; i++)
    {
        for (int j = 0; j <= msg.info.height; j++)
        {
            if(msg.data[j * msg.info.width + i] > 0) // If the cell is likely to be occupied, we'll take the point
            {
                geometry_msgs::Point p;

                p.x = i * msg.info.resolution + msg.info.origin.position.x; // msg.info.resolution + msg.info.resolution / 2 + msg.origin;
                p.y = j * msg.info.resolution + msg.info.origin.position.y;
                p.z = 0;

                points.push_back(p);
            }
        }
    }

    return points;
}

/**
 * Move the given points from one reference frame to the other.
 *
 * Iterate over the vector of points to which the transformation will be applied.
 *
 * @param points Vector containing the different points.
 * @param transform Transformation matrix that will be applied.
 * @return Vector containing the points in the new reference frame.
 */
std::vector<geometry_msgs::Point> applyTransform( const std::vector<geometry_msgs::Point>& points,
                                                  const geometry_msgs::TransformStamped& transform)
{
    std::vector<geometry_msgs::Point> result;

    for (geometry_msgs::Point p : points) // for every point
    {
        geometry_msgs::Point newPoint;
        tf2::doTransform(p, newPoint, transform); // apply transformation
        result.push_back(newPoint);
    }

    return result;
}

/**
 * Compute the distance between two points.
 *
 * @param p1 First point
 * @param p2 Second point
 * @return The distance between both points.
 */
float distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return sqrt(pow(p2.x - p1.x,2) + pow(p2.y - p1.y,2));
}

/**
 * Pinpoint the obstacles that do not belong to the map.
 *
 * Iterate over the vector of points detected by the scanner and see whether those points
 * belong to the map or not.
 *
 * @param mapPoints Vector containing the points that belong to the map.
 * @param scanPoints Vector containing the points detected by the robot.
 * @param threshold If the distance between a detected point and a map's point is shorter than
 * this threshold, the point will be discarded.
 * @return A vector containing the points that do not belong to the map (moving obstacles).
 */
std::vector<geometry_msgs::Point> scanMinusMap( const std::vector<geometry_msgs::Point>& mapPoints,
                                                const std::vector<geometry_msgs::Point>& scanPoints, 
                                                double threshold)
{
    std::vector<geometry_msgs::Point> result;

    for (geometry_msgs::Point sc : scanPoints) // for each scan point
    {
        //Check if sc is very close to a point in mapPoints
        int i = 0;
        bool close = false;
        while (i < mapPoints.size() && !close) // for each map point
        {
            if (distance(sc, mapPoints.at(i)) < threshold) // if sc is close to the map point
            {
                close = true;
            }
            i++;
        }
        if (!close) // if it is not close, then return it
            result.push_back(sc);
    }

    return result;
}

/**
 * Compute the mean of a set of points. This is the middle point.
 *
 * @param points Vector containing a set of points
 * @return A single point.
 */
geometry_msgs::Point mean(const std::vector<geometry_msgs::Point>& points)
{
    double sumX = 0.0;
    double sumY = 0.0;
    for (geometry_msgs::Point p : points)
    {
        sumX += p.x;
        sumY += p.y;
    }
    geometry_msgs::Point result;
    result.x = sumX / points.size();
    result.y = sumY / points.size();
    return result;
}

/**
 * Get the obstacles from the detected points.
 *
 * By using the mean and distance methods mentioned before, this method decides
 * whether a group of points (obstacle) turns out to be a moving obstacle or not.
 *
 * @param detections Vector containing the detected points.
 * @param threshold If the distance between the detected point and the mean of a set of points (obstacle) is
 * shorter than this threshold, the point will be part of the obstacle.
 * @return A vector of vectors containing the points that make up an obstacle (a vector containing the moving
 * obstacles).
 */
std::vector<std::vector<geometry_msgs::Point>> obstacleDetection(
        const std::vector<geometry_msgs::Point>& detections, 
        double threshold )
{
    int i = 0; // iterate over all scan points

    geometry_msgs::Point current; // detections.at(i)
    std::vector<std::vector<geometry_msgs::Point>> obstacles; // list of obstacles

    while(i < detections.size())
    {
        current = detections.at(i);

        if (obstacles.empty()) { // No obstacle registered yet
            //ROS_INFO("Created new obstacle");
            std::vector<geometry_msgs::Point> newObstacle;

            newObstacle.push_back(current); // point belongs to new obstacle

            obstacles.push_back(newObstacle); // added new obstacle
        } else {
            //ROS_INFO("Searching obstacle where it fits");
            bool fitted = false;
            int j = 0; // obstacle index
            while (j < obstacles.size() && !fitted) // for every obstacle registered
            {
                std::vector<geometry_msgs::Point> currentObstacle = obstacles.at(j);
                if (distance(current, mean(currentObstacle)) < threshold) // the point is close to the obstacle
                {
                    //ROS_INFO("Added in obstacle %d", j);
                    fitted = true;
                    currentObstacle.push_back(current); // point belongs to j-th obstacle
                }
                j++;
            }
            if (!fitted) // this point belogs to a new obstacle
            {
                //ROS_INFO("Doesn't fit in any obstacle, creating one");
                std::vector<geometry_msgs::Point> newObstacle;

                newObstacle.push_back(current); // point belongs to new obstacle

                obstacles.push_back(newObstacle); // added new obstacle
            }
        }
        i++;
    }

    return obstacles;
}

/**
 * Server obstacle_detection callback. Detect the obstacles and send them to the client.
 *
 * It performs the following operations:
 *  - get the map points
 *  - get the scan points (detected)
 *  - transform scan points to the map reference frame
 *  - discard map points close to map points
 *  - apply obstacle detection algorithm
 *  - return obstacles to client
 *
 * @param req Request.
 * @param res Response containing the obstacles.
 
 * @return a boolean value ('True' if succeeded)
 */
bool getObstacles(assignment_1::detection::Request  &req, assignment_1::detection::Response  &res)
{
    std::vector<geometry_msgs::Point> finalObstacles;

    // Get map points, listening for only one message of the /map topic
    boost::shared_ptr<nav_msgs::OccupancyGrid const> spMap;
    nav_msgs::OccupancyGrid map;
    spMap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
    if(spMap != NULL){
        map = *spMap;
    }
    std::vector<geometry_msgs::Point> mapPoints = getMapPoints(map);
    ROS_INFO("Map points:");
    printPoints(mapPoints, 5);

    // Get scan points, listening for only one message of the /scan topic
    boost::shared_ptr<sensor_msgs::LaserScan const> spScan;
    sensor_msgs::LaserScan scan;
    spScan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
    if(spScan != NULL)
    {
        scan = *spScan;
    }
    std::vector<geometry_msgs::Point> scanPoints = getScanPoints(scan);
    ROS_INFO("Scan points, before transform:");
    printPoints(scanPoints, 5);

    // Get transform between the robot scan frame (source) and the map frame (target)
    // The frame of the robot scan can be found with "rostopic echo /scan", it is "base_laser_link"
    // The frame of the map can be found with "rostopic echo /map", it is "map"
    tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformRobot2Map;

    try{
        transformRobot2Map = tfBuffer.lookupTransform(
                "map", //target frame
                "base_laser_link", //source frame
                ros::Time(0),
                ros::Duration(5.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could NOT find transform: %s", ex.what());
        return -1;
    }

    // Apply the transform to all scan points
    scanPoints = applyTransform(scanPoints, transformRobot2Map);
    ROS_INFO("Scan points, after transform:");
    printPoints(scanPoints, 5);

    // Discard scan points close to a map point
    std::vector<geometry_msgs::Point> scanMinMap = scanMinusMap(mapPoints, scanPoints, req.scanMinusMapThreshold);
    ROS_INFO("Scan minus map with threshold 0.5:");
    printPoints(scanMinMap, 5);

    // Apply the obstacle detection algorithm
    std::vector<std::vector<geometry_msgs::Point>> obstacles = obstacleDetection(scanMinMap, req.obstacleDetectionThreshold);
    for (std::vector<geometry_msgs::Point> obs : obstacles)
    {
        geometry_msgs::Point obsPoint = mean(obs); // this is the obstacle
        ROS_INFO("Obstacle detected in (%4.2f, %4.2f)", obsPoint.x, obsPoint.y); // print

        finalObstacles.push_back(obsPoint); // to send to client
    }

    res.obsPoint = finalObstacles;

    return true;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection_node");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("obstacle_detection", getObstacles);
    ROS_INFO("Server ready to give obstacles");

    ros::spin();

    return 0;
}