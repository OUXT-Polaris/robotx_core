#ifndef CARROT_PLANNER_H_INCLUDED
#define CARROT_PLANNER_H_INCLUDED

//headers in STL
#include <mutex>

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

class carrot_planner
{
public:
    carrot_planner();
    ~carrot_planner();
    void run();
private:
    void _linear_velocity_callback(const std_msgs::Float64::ConstPtr msg);
    void _torelance_callback(const std_msgs::Float64::ConstPtr msg);
    void _goal_pose_callback(geometry_msgs::PoseStamped msg);
    std::string _goal_topic;
    std::string _tolerance_topic;
    std::string _robot_frame;
    std::string _map_frame;
    std::string _linear_velocity_topic;
    std::string _twist_topic;
    double _linear_velocity;
    double _torelance;
    double _publish_rate;
    volatile bool _torelance_recieved;
    volatile bool _goal_recieved;
    ros::NodeHandle _nh;
    ros::Subscriber _tolerance_sub;
    ros::Subscriber _goal_pose_sub;
    ros::Publisher _twist_pub;
    geometry_msgs::PoseStamped _goal_pose;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    std::mutex _mtx;
};
#endif  //CARROT_PLANNER_H_INCLUDED