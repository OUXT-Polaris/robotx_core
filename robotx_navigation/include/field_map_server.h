#ifndef FIELD_MAP_SERVER_H_INCLUDED
#define FIELD_MAP_SERVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>

//headers in this package
#include <robotx_msgs/FieldMap.h>

//headers in STL
#include <fstream>

class field_map_server
{
public:
    field_map_server(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~field_map_server();
    void run();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher field_map_pub_;
    ros::Publisher marker_pub_;
    robotx_msgs::FieldMap field_map_;
    std::string map_frame_;
    void load_();
    std::vector<std::string> split_(std::string& input, char delimiter);
    visualization_msgs::MarkerArray markers_;
};

#endif  //FIELD_MAP_SERVER_H_INCLUDED