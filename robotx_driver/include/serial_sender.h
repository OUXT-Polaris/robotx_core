#ifndef SERIAL_SENDER_H_INCLUDED
#define SERIAL_SENDER_H_INCLUDED

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace boost::asio;

class serial_sender
{
public:
    serial_sender();
    ~serial_sender();
    void run();
private:
    void message_callback_(const std_msgs::StringConstPtr msg);
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string port_;
    std::string write_buf_;
    int baud_rate_;
    double send_rate_;
    void send_();
    ros::Subscriber message_sub_;
};
#endif  //SERIAL_SENDER_H_INCLUDED