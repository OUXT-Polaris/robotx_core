#ifndef ROSTATE_MACHINE_H_INCLUDED
#define ROSTATE_MACHINE_H_INCLUDED

//headers in robotx_packages
#include "state_machine.h"
#include <robotx_msgs/State.h>

//headers in STL
#include <memory>

//headers in ROS
#include <ros/ros.h>

class rostate_machine
{
public:
    rostate_machine(std::string xml_filepath, std::string dot_filepath, std::string state_machine_name);
    ~rostate_machine();
private:
    void publish_current_state_();
    std::shared_ptr<state_machine> state_machine_ptr_;
    ros::Publisher current_state_pub_;
    std::string state_machine_name_;
};

#endif  //ROSTATE_MACHINE_H_INCLUDED