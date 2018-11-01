// headers for ros
#include <ros/ros.h>

//headers in this package
#include <control_state_machine.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robotx_decision_maker_node");
    control_state_machine control_state_machine_;
    ros::spin();
    return 0;
}