#ifndef BATTERY_MONITOR_DRIVER_H_INCLUDED
#define BATTERY_MONITOR_DRIVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

//headers in this package
#include <tcp_server.h>

//headers in STL
#include <map>

//headers in boost
#include <boost/shared_ptr.hpp>

class battery_monitor_driver
{
public:
    battery_monitor_driver(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~battery_monitor_driver();
private:
    void publish_battery_state_();
    boost::asio::io_service io_service_;
    std::string ip_address_;
    int port_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::map<std::string,ros::Publisher> battery_state_pubs_;
    boost::shared_ptr<tcp_server> tcp_server_ptr_;
};
#endif  //BATTERY_MONITOR_DRIVER_H_INCLUDED