#include <robotx_task_planner.h>

robotx_task_planner::robotx_task_planner()
{
    ros::param::param<std::string>(ros::this_node::getName() + "/current_state_topic", current_state_topic_, "/robotx_state_machine_node/mission_state_machine/current_state");
    ros::param::param<std::string>(ros::this_node::getName() + "/trigger_event_topic", trigger_event_topic_, "/robotx_state_machine_node/mission_state_machine/trigger_event");
    trigger_event_pub_ = nh_.advertise<robotx_msgs::Event>(trigger_event_topic_, 10);
    current_state_sub_ = nh_.subscribe(current_state_topic_, 10, &robotx_task_planner::current_state_callback_, this);
}

robotx_task_planner::~robotx_task_planner()
{

}

void robotx_task_planner::current_state_callback_(const robotx_msgs::StateConstPtr msg)
{
    return;
}