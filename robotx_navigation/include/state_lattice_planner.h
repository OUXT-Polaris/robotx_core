#ifndef STATE_LATTICE_PLANNER_H_INCLUDED
#define STATE_LATTICE_PLANNER_H_INCLUDED

#include <geometry_msgs/Twist.h>

class state_lattice_planner
{
public:
    state_lattice_planner();
    ~state_lattice_planner();
    bool plan(geometry_msgs::Twist current_twist);
private:

};
#endif  //STATE_LATTICE_PLANNER_H_INCLUDED