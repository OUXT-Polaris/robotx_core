#ifndef STATE_LATTICE_PLANNER_H_INCLUDED
#define STATE_LATTICE_PLANNER_H_INCLUDED

//headers in ROS
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

//headers in FLANN
#include <flann/flann.hpp>

//headers in boost
#include <boost/shared_ptr.hpp>

enum planner_result
{
    NO_OBSTACLE = 0,
    SUCCEED_TO_FIND_PATH = 1,
    FAILED_TO_FIND_PATH = 2
};

class state_lattice_planner
{
public:
    state_lattice_planner();
    ~state_lattice_planner();
    planner_result plan(geometry_msgs::Twist raw_twist_cmd, geometry_msgs::PoseStamped target_pose, 
        geometry_msgs::Twist current_twist, boost::shared_ptr<nav_msgs::OccupancyGrid> map_ptr, geometry_msgs::Twist& twist_cmd);
private:
};
#endif  //STATE_LATTICE_PLANNER_H_INCLUDED