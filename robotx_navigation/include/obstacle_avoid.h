#ifndef OBSTACLE_AVOID_H_INCLUDED
#define OBSTACLE_AVOID_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>

//headers in STL
#include <mutex>

//headers in boost
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

//headers in robotx_packages
#include <robotx_msgs/State.h>
#include <robotx_msgs/Event.h>
#include <robotx_msgs/ObstacleMap.h>
#include <state_lattice_planner.h>

class obstacle_avoid
{
public:
    obstacle_avoid();
    ~obstacle_avoid();
private:
    ros::NodeHandle nh_;
    ros::Subscriber twist_cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber target_pose_sub_;
    ros::Subscriber current_state_sub_;
    ros::Publisher trigger_event_pub_;
    ros::Publisher twist_cmd_pub_;
    ros::Publisher nearest_obstacle_range_pub_;
    void publish_nearest_obstacle_range_();
    void current_state_callback_(robotx_msgs::State msg);
    void odom_callback_(const nav_msgs::Odometry::ConstPtr msg);
    void obstacle_map_callback_(const robotx_msgs::ObstacleMap::ConstPtr msg);
    void target_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg);
    bool obstacle_found_();
    geometry_msgs::PoseStamped transformed_target_pose_;
    robotx_msgs::ObstacleMap map_;
    boost::optional<geometry_msgs::PoseStamped> target_pose_;
    volatile bool odom_recieved_;
    volatile bool twist_cmd_recieved_;
    volatile bool map_recieved_;
    nav_msgs::Odometry odom_;
    std::mutex mtx_;
    std::string map_topic_;
    std::string raw_cmd_vel_topic_;
    std::string cmd_vel_topic_;
    std::string odom_topic_;
    std::string target_pose_topic_;
    std::string current_state_topic_;
    std::string trigger_event_topic_;
    std::string robot_frame_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    boost::optional<robotx_msgs::State> current_state_;
    double search_angle_;
    double search_radius_;
    double search_radius_side_;
    double search_angle_behind_;
    double search_radius_behind_;
    state_lattice_planner planner_;
};

#endif  //OBSTACLE_AVOID_H_INCLUDED