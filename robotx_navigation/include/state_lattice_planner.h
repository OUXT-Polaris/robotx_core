#ifndef STATE_LATTICE_PLANNER
#define STATE_LATTICE_PLANNER

//headers in ROS
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

//headers in this message
#include <robotx_msgs/ObstacleMap.h>

//headers in boost
#include <boost/optional.hpp>

struct state_lattice_parameters
{
    double max_angular_acceleration;
    double min_angular_acceleration;
    double max_linear_acceleration;
    double min_linear_acceleration;
    double max_linear_velocity;
    double min_linear_velocity;
    double max_angular_velocity;
    int num_predictions;
    int num_samples_angular;
    int num_samples_linear;
};

class state_lattice_planner
{
public:
    state_lattice_planner(state_lattice_parameters params);
    ~state_lattice_planner();
    boost::optional<std::pair<nav_msgs::Path,geometry_msgs::Twist> > plan(robotx_msgs::ObstacleMap map, geometry_msgs::Twist current_twist);
private:
    state_lattice_parameters params_;
};

#endif  //STATE_LATTICE_PLANNER