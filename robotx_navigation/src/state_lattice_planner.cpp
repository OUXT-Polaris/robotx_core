#include <state_lattice_planner.h>

state_lattice_planner::state_lattice_planner()
{

}

state_lattice_planner::~state_lattice_planner()
{

}

planner_result state_lattice_planner::plan(geometry_msgs::Twist raw_twist_cmd, geometry_msgs::PoseStamped target_pose, 
    geometry_msgs::Twist current_twist, boost::shared_ptr<nav_msgs::OccupancyGrid> map_ptr, geometry_msgs::Twist& twist_cmd)
{
    bool obstacle_found = false;
    for(auto data_itr = map_ptr->data.begin(); data_itr != map_ptr->data.end(); data_itr++)
    {
        if(*data_itr > 0)
        {
            obstacle_found = true;
        }
    }
    if(obstacle_found == false)
    {
        twist_cmd = raw_twist_cmd;
        return NO_OBSTACLE;
    }
}