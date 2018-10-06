#ifndef WAYPOINT_SERVER_H_INCLUDED
#define WAYPOINT_SERVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

//headers in robotx_packages
#include <robotx_msgs/WayPointArray.h>

class waypoint_server
{
public:
    waypoint_server();
    ~waypoint_server();
private:
    ros::NodeHandle nh_;
    std::string waypoint_bag_file_path_;
    robotx_msgs::WayPointArray waypoints_;
};
#endif  //WAYPOINT_SERVER_H_INCLUDED