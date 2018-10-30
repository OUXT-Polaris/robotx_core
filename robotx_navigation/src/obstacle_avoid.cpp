#include <obstacle_avoid.h>

obstacle_avoid::obstacle_avoid()
{
    twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    map_recieved_ = false;
    odom_recieved_ = false;
    twist_cmd_recieved_ = false;
    nh_.param<std::string>(ros::this_node::getName()+"/map_topic", map_topic_, "/obstacle_map");
    nh_.param<std::string>(ros::this_node::getName()+"/raw_cmd_vel_topic", raw_cmd_vel_topic_, ros::this_node::getName()+"/input_cmd_vel");
    nh_.param<std::string>(ros::this_node::getName()+"/odom_topic", odom_topic_, "/odom");
    nh_.param<std::string>(ros::this_node::getName()+"/target_pose_topic", target_pose_topic_, ros::this_node::getName()+"/target_pose");
    map_sub_ = nh_.subscribe(map_topic_, 3, &obstacle_avoid::obstacle_map_callback_, this);
    twist_cmd_sub_ = nh_.subscribe(raw_cmd_vel_topic_, 10, &obstacle_avoid::twist_cmd_callback_, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &obstacle_avoid::odom_callback_, this);
    target_pose_sub_ = nh_.subscribe(target_pose_topic_, 10, &obstacle_avoid::target_pose_callback_, this);
}

obstacle_avoid::~obstacle_avoid()
{

}

void obstacle_avoid::target_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    target_pose_ = *msg;
    return;
}

void obstacle_avoid::twist_cmd_callback_(const geometry_msgs::Twist::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    twist_cmd_recieved_ = true;
    raw_twist_cmd_ = *msg;
    return;
}

void obstacle_avoid::odom_callback_(const nav_msgs::Odometry::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    odom_recieved_ = true;
    odom_ = *msg;
    if(odom_recieved_ && map_recieved_ && twist_cmd_recieved_)
    {
        geometry_msgs::Twist twist_cmd;
        planner_.plan(raw_twist_cmd_, target_pose_, odom_, map_ptr_, twist_cmd);
        twist_cmd_pub_.publish(raw_twist_cmd_);
    }
    return;
}

void obstacle_avoid::obstacle_map_callback_(const nav_msgs::OccupancyGrid::Ptr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    map_ptr_ = boost::shared_ptr<nav_msgs::OccupancyGrid>(msg);
    map_recieved_ = true;
    return;
}