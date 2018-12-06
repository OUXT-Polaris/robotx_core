#ifndef SIGNAL_LAMP_DRIVER_H_INCLUDED
#define SIGNAL_LAMP_DRIVER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>

//headers in this package
#include <robotx_msgs/SignalLamp.h>
#include <tcp_client.h>

//headers in boost
#include <boost/thread.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

//headers in STL
#include <mutex>
#include <sstream>

class singal_lamp_driver
{
public:
    singal_lamp_driver(ros::NodeHandle nh, ros::NodeHandle pnh);
    ~singal_lamp_driver();
    void run();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    void command_callback_(const robotx_msgs::SignalLamp::ConstPtr msg);
    double timeout_;
    double publish_rate_;
    std::string cmd_topic_;
    std::string ip_address_;
    int port_;
    void publish_cmd_();
    boost::optional<robotx_msgs::SignalLamp> command_;
    ros::Subscriber command_sub_;
    boost::shared_ptr<tcp_client> tcp_client_ptr_;
    boost::asio::io_service io_service_;
    std::mutex mtx_;
};

#endif  //SIGNAL_LAMP_DRIVER_H_INCLUDED