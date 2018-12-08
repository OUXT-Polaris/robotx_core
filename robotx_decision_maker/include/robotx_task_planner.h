#ifndef ROBOTX_TASK_PLANNER_H_INCLUDED
#define ROBOTX_TASK_PLANNER_H_INCLUDED

//headers in ROSc
#include <ros/ros.h>

//headers in this package
#include <robotx_msgs/State.h>
#include <robotx_msgs/Event.h>

class robotx_task_planner
{
public:
    robotx_task_planner();
    ~robotx_task_planner();
private:
    void current_state_callback_(const robotx_msgs::StateConstPtr msg);
    ros::NodeHandle nh_;
    ros::Publisher trigger_event_pub_;
    ros::Subscriber waypoint_planner_cmd_sub_;
    ros::Subscriber obstacle_avoid_cmd_sub_;
    ros::Subscriber current_state_sub_;
    std::string current_state_topic_;
    std::string trigger_event_topic_;
};

#endif  //ROBOTX_TASK_PLANNER_H_INCLUDED