#ifndef OBSTACLE_AVOID_H_INCLUDED
#define OBSTACLE_AVOID_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

//headers in this package
#include <state_lattice_planner.h>

class obstacle_avoid
{
public:
    obstacle_avoid();
    ~obstacle_avoid();
private:
    ros::NodeHandle nh_;
    ros::Subscriber twist_cmd_sub_;
    ros::Publisher twist_cmd_pub_;
    void twist_cmd_callback_(const geometry_msgs::Twist::ConstPtr msg);
    void obstacle_map_callback_(const nav_msgs::OccupancyGrid::Ptr msg);
    state_lattice_planner planner_;
    boost::shared_ptr<nav_msgs::OccupancyGrid> map_ptr_;
};

#endif  //OBSTACLE_AVOID_H_INCLUDED