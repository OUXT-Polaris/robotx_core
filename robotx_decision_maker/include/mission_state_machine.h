#ifndef MISSION_STATE_MACHINE_H_INCLUDED
#define MISSION_STATE_MACHINE_H_INCLUDED

//headers in this package
#include <rostate_machine.h>

//headers in ROS
#include <ros/ros.h>
#include <ros/package.h>

//headers in STL
#include <memory>

class mission_state_machine
{
public:
    mission_state_machine();
    ~mission_state_machine();
    void run();
private:
    std::shared_ptr<rostate_machine> state_machine_ptr_;
};

#endif  //MISSION_STATE_MACHINE_H_INCLUDED