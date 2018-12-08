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
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/optional.hpp>

//headers in robotx_package
#include <robotx_msgs/State.h>
#include <robotx_msgs/Event.h>
#include <robotx_msgs/CarrotPlannerConfigure.h>

class carrot_planner
{
public:
    carrot_planner();
    ~carrot_planner();
    void run();
private:
    double get_diff_angle_(double from,double to);
    double _get_diff_yaw_to_target();
    double _get_diff_yaw();
    void _robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg);
    void _current_state_callback(const robotx_msgs::State::ConstPtr msg);
    void _goal_pose_callback(geometry_msgs::PoseStamped msg);
    void _configure_callback(const robotx_msgs::CarrotPlannerConfigure::ConstPtr msg);
    void _publish_twist_cmd();
    std::string _goal_topic;
    std::string _robot_frame;
    std::string _map_frame;
    std::string _twist_topic;
    std::string _robot_pose_topic;
    robotx_msgs::CarrotPlannerConfigure _configure;
    double _linear_velocity;
    double _angular_velocity;
    double _torelance;
    double _angular_tolerance;
    double _publish_rate;
    bool _enable_backword;
    volatile bool _torelance_recieved;
    volatile bool _goal_recieved;
    ros::NodeHandle _nh;
    ros::Subscriber _tolerance_sub;
    ros::Subscriber _goal_pose_sub;
    ros::Subscriber _linear_velocity_sub;
    ros::Subscriber _current_stete_sub;
    ros::Subscriber _state_sub;
    ros::Subscriber _robot_pose_sub;
    ros::Subscriber _configure_sub;
    ros::Publisher _twist_pub;
    ros::Publisher _trigger_event_pub;
    geometry_msgs::PoseStamped _goal_pose;
    geometry_msgs::Pose2D _goal_pose_2d;
    geometry_msgs::Pose2D _robot_pose_2d;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    std::mutex _mtx;
    boost::optional<robotx_msgs::State> _current_state;
};
#endif  //CARROT_PLANNER_H_INCLUDED