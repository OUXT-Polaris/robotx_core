#include <state_lattice_planner.h>

state_lattice_planner::state_lattice_planner(state_lattice_parameters params)
{
    params_ = params;
}

state_lattice_planner::~state_lattice_planner()
{

}

boost::optional<std::pair<nav_msgs::Path,geometry_msgs::Twist>> state_lattice_planner::plan(robotx_msgs::ObstacleMap map, geometry_msgs::Twist current_twist)
{
    std::pair<nav_msgs::Path,geometry_msgs::Twist> ret;
    for(int i=0; i< params_.num_samples_angular; i++)
    {
        for(int m=0; m<params_.num_samples_linear; m++)
        {
            double linear_acceleration = current_twist.linear.x;
            double angular_acceleration = current_twist.angular.z;
        }
    }
    return ret;
}

nav_msgs::Path state_lattice_planner::generate_path(geometry_msgs::Twist current_twist, double linear_acceleration, double angular_acceleration)
{
    geometry_msgs::Twist twist;
    twist = current_twist;
    nav_msgs::Path path;
    for(int i=0; i<params_.num_predictions; i++)
    {

    }
    return path;
}

double state_lattice_planner::evaluate_function_(robotx_msgs::ObstacleMap map,nav_msgs::Path path)
{
    return 0;
}