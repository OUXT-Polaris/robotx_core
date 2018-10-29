#include <state_lattice_planner.h>

state_lattice_planner::state_lattice_planner()
{

}

state_lattice_planner::~state_lattice_planner()
{

}

bool state_lattice_planner::plan(geometry_msgs::Twist current_twist, boost::shared_ptr<nav_msgs::OccupancyGrid> map_ptr)
{
    return true;
}