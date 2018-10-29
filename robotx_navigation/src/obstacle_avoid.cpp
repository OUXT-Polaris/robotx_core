#include <obstacle_avoid.h>

obstacle_avoid::obstacle_avoid()
{
    twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    twist_cmd_sub_ = nh_.subscribe(ros::this_node::getName()+"/input_cmd_vel", 10, &obstacle_avoid::twist_cmd_callback_, this);
}

obstacle_avoid::~obstacle_avoid()
{

}

void obstacle_avoid::twist_cmd_callback_(const geometry_msgs::Twist::ConstPtr msg)
{
    geometry_msgs::Twist twist_cmd = *msg;
    twist_cmd_pub_.publish(twist_cmd);
    return;
}

void obstacle_avoid::obstacle_map_callback_(const nav_msgs::OccupancyGrid::Ptr msg)
{
    map_ptr_ = boost::shared_ptr<nav_msgs::OccupancyGrid>(msg);
    return;
}