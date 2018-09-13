#ifndef ROBOTX_PATH_PLANNER_H_INCLUDED
#define ROBOTX_PATH_PLANNER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
    ros::Subscriber _euclidean_cluster_sub,_robot_pose_sub;
    ros::Publisher _marker_pub;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    void _euclidean_cluster_callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr msg);
    void _pose_callback(const geometry_msgs::PoseStampedConstPtr msg);
    boost::shared_ptr<euclidean_cluster_buffer> _buffer;
    double _max_cluster_length;
    double _min_cluster_length;
    double _inflation_radius;
};
#endif  //ROBOTX_PATH_PLANNER_H_INCLUDED