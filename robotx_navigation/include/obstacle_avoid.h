#ifndef OBSTACLE_AVOID_H_INCLUDED
#define OBSTACLE_AVOID_H_INCLUDED

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

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
};

#endif  //OBSTACLE_AVOID_H_INCLUDED