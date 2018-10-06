#ifndef WAYPOINT_LOGGER_H_INCLUDED
#define WAYPOINT_LOGGER_H_INCLUDED

//headers in robotx_packages
#include <robotx_msgs/WayPointArray.h>
#include <robotx_msgs/WayPoint.h>

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosbag/bag.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//headers in STL
#include <mutex>

//headers in boost
#include <boost/shared_ptr.hpp>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::NavSatFix> sync_polycy;

class waypoint_logger
{
public:
    waypoint_logger();
    ~waypoint_logger();
private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped> > pose_sub_;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix> > fix_sub_;
    boost::shared_ptr<message_filters::Synchronizer<sync_polycy> > sync_;
    ros::Publisher marker_pub_;
    void joy_callback_(const sensor_msgs::Joy::ConstPtr msg);
    void pose_fix_callback_(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::NavSatFixConstPtr& fix_msg);
    robotx_msgs::WayPointArray waypoints_;
    std::string joy_topic_,pose_topic_,fix_topic_,map_frame_;
    int joy_button_index_;
    uint8_t waypoint_id_;
    volatile bool button_pressed_;
    volatile bool waypoint_recieved_;
    std::mutex mutex_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    robotx_msgs::WayPoint last_waypoint_;
};
#endif  //WAYPOINT_LOGGER_H_INCLUDED