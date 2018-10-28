#ifndef DYNAMIXEL_DRIVER_H_INCLUDED
#define DYNAMIXEL_DRIVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

class dynamixel_driver
{
public:
    dynamixel_driver();
    ~dynamixel_driver();
private:
    std::string input_topic_;
    ros::NodeHandle nh_;
    int motor_id_;
    void command_callback_(std_msgs::Float64 msg);
    ros::ServiceClient joint_command_client_;
};

#endif  //DYNAMIXEL_DRIVER_H_INCLUDED