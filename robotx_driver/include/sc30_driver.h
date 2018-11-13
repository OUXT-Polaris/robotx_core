#ifndef SC30_DRIVER_H_INCLUDED
#define SC30_DRIVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/NavSatFix.h>

class sc30_driver
{
public:
    sc30_driver(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~sc30_driver();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void nmea_cakkback_(const nmea_msgs::Sentence::ConstPtr msg);
    ros::Subscriber nmea_sub_;
    ros::Publisher navsat_fix_pub_;
};

#endif  //SC30_DRIVER_H_INCLUDED