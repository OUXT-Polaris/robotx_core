#ifndef ROSTATE_MACHINE_H_INCLUDED
#define ROSTATE_MACHINE_H_INCLUDED

//headers in robotx_packages
#include "state_machine.h"
#include <robotx_msgs/State.h>
#include <robotx_msgs/Event.h>

//headers in STL
#include <memory>

//headers in ROS
#include <ros/ros.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class rostate_machine
{
public:
    rostate_machine(std::string xml_filepath, std::string dot_filepath, std::string state_machine_name);
    ~rostate_machine();
    void run();
private:
    void publish_current_state_();
    void event_callback_(robotx_msgs::Event msg);
    std::shared_ptr<state_machine> state_machine_ptr_;
    ros::NodeHandle nh_;
    ros::Publisher current_state_pub_;
    ros::Subscriber trigger_event_sub_;
    std::string state_machine_name_;
    double publish_rate_;
};

#endif  //ROSTATE_MACHINE_H_INCLUDED