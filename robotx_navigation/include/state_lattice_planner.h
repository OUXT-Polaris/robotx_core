#ifndef STATE_LATTICE_PLANNER_H_INCLUDED
#define STATE_LATTICE_PLANNER_H_INCLUDED

//headers in ROS
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

//headers in FLANN
#include <flann/flann.hpp>

//headers in boost
#include <boost/shared_ptr.hpp>

class state_lattice_planner
{
public:
    state_lattice_planner();
    ~state_lattice_planner();
    bool plan(geometry_msgs::Twist current_twist, boost::shared_ptr<nav_msgs::OccupancyGrid> map_ptr);
private:
};
#endif  //STATE_LATTICE_PLANNER_H_INCLUDED