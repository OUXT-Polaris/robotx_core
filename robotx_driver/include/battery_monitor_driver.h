#ifndef BATTERY_MONITOR_DRIVER_H_INCLUDED
#define BATTERY_MONITOR_DRIVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

class battery_monitor_driver
{
public:
    battery_monitor_driver(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~battery_monitor_driver();
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
};
#endif  //BATTERY_MONITOR_DRIVER_H_INCLUDED