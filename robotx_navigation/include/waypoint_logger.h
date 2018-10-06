#ifndef WAYPOINT_LOGGER_H_INCLUDED
#define WAYPOINT_LOGGER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

//headers in STL
#include <mutex>

class waypoint_logger
{
public:
    waypoint_logger();
    ~waypoint_logger();
private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher marker_pub_;
    void joy_callback_(const sensor_msgs::Joy::ConstPtr msg);
    void pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg);
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    geometry_msgs::PoseStamped last_pose_;
    std::string joy_topic_,pose_topic_;
    std::mutex mutex_;
};
#endif  //WAYPOINT_LOGGER_H_INCLUDED