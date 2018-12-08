#ifndef BATTERY_MONITOR_DRIVER_H_INCLUDED
#define BATTERY_MONITOR_DRIVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

//headers in this package
#include <tcp_server.h>
#include <robotx_msgs/PowerStatus.h>

//headers in boost
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

class battery_monitor_driver
{
public:
    battery_monitor_driver(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~battery_monitor_driver();
private:
    void publish_battery_state_(std::string data);
    boost::asio::io_service io_service_;
    std::string ip_address_;
    int port_;
    ros::Publisher control_battery_pub_;
    ros::Publisher motor_battery_pub_;
    ros::Publisher power_state_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    boost::shared_ptr<tcp_server> tcp_server_ptr_;
};
#endif  //BATTERY_MONITOR_DRIVER_H_INCLUDED