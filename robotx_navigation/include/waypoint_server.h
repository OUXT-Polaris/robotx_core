#ifndef WAYPOINT_SERVER_H_INCLUDED
#define WAYPOINT_SERVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <robotx_msgs/State.h>
#include <robotx_msgs/Event.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

//headers in stl
#include <mutex>
#include <fstream>
#include <sstream>

//headers in boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/optional.hpp>

class waypoint_server
{
public:
    waypoint_server();
    ~waypoint_server();
private:
    ros::NodeHandle nh_;
    std::string waypoint_csv_file_path_,robot_frame_,map_frame_,robot_pose_topic_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    ros::Subscriber navigation_status_sub_;
    ros::Subscriber robot_pose_sub_;
    ros::Publisher marker_pub_;
    ros::Publisher waypoint_pub_;
    ros::Publisher trigger_event_pub_;
    std::mutex mutex_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    int target_waypoint_index_;
    volatile bool first_waypoint_finded_;
    void publish_marker_();
    void robot_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg);
    void navigation_status_callback_(robotx_msgs::State msg);
    bool load_next_waypoint_();
    boost::optional<int> get_nearest_wayppoint_(const geometry_msgs::PoseStamped::ConstPtr msg);
    std::vector<std::string> split_(std::string& input, char delimiter);
};
#endif  //WAYPOINT_SERVER_H_INCLUDED