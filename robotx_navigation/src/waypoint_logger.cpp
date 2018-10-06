#include <waypoint_logger.h>

waypoint_logger::waypoint_logger()
{
    nh_.param<std::string>(ros::this_node::getName()+"/joy_topic", joy_topic_, "/joy");
    nh_.param<std::string>(ros::this_node::getName()+"/pose_topic", pose_topic_, "/robot_pose");
    joy_sub_ = nh_.subscribe(joy_topic_, 1 ,&waypoint_logger::joy_callback_, this);
    pose_sub_ = nh_.subscribe(pose_topic_, 1 ,&waypoint_logger::pose_callback_, this);
}

waypoint_logger::~waypoint_logger()
{

}

void waypoint_logger::joy_callback_(const sensor_msgs::Joy::ConstPtr msg)
{
    mutex_.lock();
    mutex_.unlock();
    return;
}

void waypoint_logger::pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    mutex_.lock();
    last_pose_ = *msg;
    mutex_.unlock();
    return;
}