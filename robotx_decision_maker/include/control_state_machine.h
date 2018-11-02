#ifndef CONTROL_STATE_MACHINE_H_INCLUDED
#define CONTROL_STATE_MACHINE_H_INCLUDED

//headers in this package
#include <state_machine.h>

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

//headers in STL
#include <memory>

//headers in boost
#include <boost/thread.hpp>

class control_state_machine
{
public:
    control_state_machine();
    ~control_state_machine();
private:

    std::shared_ptr<state_machine> state_machine_ptr_;
    ros::Publisher dot_pub_;
    ros::NodeHandle nh_;
};
#endif  //CONTROL_STATE_MACHINE_H_INCLUDED