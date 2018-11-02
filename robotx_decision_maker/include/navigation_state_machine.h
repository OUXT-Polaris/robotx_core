#ifndef NAVIGATION_STATE_MACHINE_H_INCLUDED
#define NAVIGATION_STATE_MACHINE_H_INCLUDED

//headers in this package
#include <rostate_machine.h>

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>

//headers in STL
#include <memory>

class navigation_state_machine
{
public:
    navigation_state_machine();
    ~navigation_state_machine();
private:
    std::shared_ptr<rostate_machine> state_machine_ptr_;
};

#endif  //NAVIGATION_STATE_MACHINE_H_INCLUDED