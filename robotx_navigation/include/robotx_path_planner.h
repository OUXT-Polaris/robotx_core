#ifndef ROBOTX_PATH_PLANNER_H_INCLUDED
#define ROBOTX_PATH_PLANNER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

class robotx_path_planner
{
public:
    robotx_path_planner();
    ~robotx_path_planner();
private:
    ros::NodeHandle _nh;
    ros::Subscriber _euclidean_cluster_sub;
};
#endif  //ROBOTX_PATH_PLANNER_H_INCLUDED