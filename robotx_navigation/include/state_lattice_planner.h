#ifndef STATE_LATTICE_PLANNER
#define STATE_LATTICE_PLANNER

//headers in ROS
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_listener.h>

//headers in this message
#include <robotx_msgs/ObstacleMap.h>
#include <robotx_msgs/ObstacleAvoidConfigure.h>

//headers in boost
#include <boost/optional.hpp>

class state_lattice_planner
{
public:
    state_lattice_planner(robotx_msgs::ObstacleAvoidConfigure params);
    ~state_lattice_planner();
    boost::optional<geometry_msgs::Twist> plan(robotx_msgs::ObstacleMap map, nav_msgs::Odometry odom, geometry_msgs::Pose2D target_pose);
    void update_params(robotx_msgs::ObstacleAvoidConfigure params);
private:
    robotx_msgs::ObstacleAvoidConfigure params_;
    std::vector<geometry_msgs::Pose2D> generate_path(nav_msgs::Odometry odom, double linear_acceleration, double angular_acceleration);
    double get_nearest_obstacle_distance_(robotx_msgs::ObstacleMap map, std::vector<geometry_msgs::Pose2D> path);
    double evaluate_function_(double nearest_obstacle_distance, geometry_msgs::Pose2D end_pose, geometry_msgs::Pose2D target_pose);
};

#endif  //STATE_LATTICE_PLANNER