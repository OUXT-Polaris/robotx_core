#ifndef CARROT_PLANNER_H_INCLUDED
#define CARROT_PLANNER_H_INCLUDED

#include <ros/ros.h>

class carrot_planner
{
public:
    carrot_planner();
    ~carrot_planner();
private:
    std::string _map_topic;
};
#endif  //CARROT_PLANNER_H_INCLUDED