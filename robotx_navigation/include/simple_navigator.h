#ifndef SIMPLE_NAVIGATOR_H_INCLUDED
#define SIMPLE_NAVIGATOR_H_INCLUDED

#include <ros/ros.h>

class simple_navigator
{
public:
    simple_navigator();
    ~simple_navigator();
private:
    ros::NodeHandle _nh;
    double _min_search_radius;
    double _max_search_radius;
};
#endif  //SIMPLE_NAVIGATOR_H_INCLUDED