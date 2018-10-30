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
    NO_OBSTACLE_ON_CURRENT_PATH = 1,
    SUCCEED_TO_FIND_PATH = 2,
    FAILED_TO_FIND_PATH = 3
};

class state_lattice_planner
{
public:
    state_lattice_planner();
    ~state_lattice_planner();
    void set_parameters(double max_angular_vel, double max_angular_acceleration,
        double max_linear_vel, double max_linear_acceleration, double prediction_time, int num_prediction);
    planner_result plan(geometry_msgs::Twist raw_twist_cmd, geometry_msgs::PoseStamped target_pose, 
        geometry_msgs::Twist current_twist, boost::shared_ptr<nav_msgs::OccupancyGrid> map_ptr, geometry_msgs::Twist& twist_cmd);
private:
    boost::optional<flann::Matrix<float> > map_to_matrix_(boost::shared_ptr<nav_msgs::OccupancyGrid> map_ptr);
    double max_angular_vel_;
    double max_angular_acceleration_;
    double max_linear_vel_;
    double max_linear_acceleration_;
    double prediction_time_;
    int num_prediction_;
};
#endif  //STATE_LATTICE_PLANNER_H_INCLUDED