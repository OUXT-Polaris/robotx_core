// headers for ros
#include <ros/ros.h>

//headers in this package
#include <control_state_machine.h>
#include <mission_state_machine.h>
#include <navigation_state_machine.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robotx_decision_maker_node");
    control_state_machine control_state_machine_;
    mission_state_machine mission_state_machine_;
    navigation_state_machine navigation_state_machine_;
    control_state_machine_.run();
    mission_state_machine_.run();
    navigation_state_machine_.run();
    ros::spin();
    return 0;
}