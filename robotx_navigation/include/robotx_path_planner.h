#ifndef ROBOTX_PATH_PLANNER_H_INCLUDED
#define ROBOTX_PATH_PLANNER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

//headers in this package
#include <euclidean_cluster_buffer.h>

//headers in boost
#include <boost/shared_ptr.hpp>

class robotx_path_planner
{
public:
    robotx_path_planner();
    ~robotx_path_planner();
private:
    ros::NodeHandle _nh;
    ros::Subscriber _euclidean_cluster_sub;
    void _euclidean_cluster_callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr msg);
    boost::shared_ptr<euclidean_cluster_buffer> _buffer;
    double _max_cluster_length;
    double _min_cluster_length;
};
#endif  //ROBOTX_PATH_PLANNER_H_INCLUDED