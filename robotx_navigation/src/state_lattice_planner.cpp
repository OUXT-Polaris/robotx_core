#include <state_lattice_planner.h>

state_lattice_planner::state_lattice_planner(state_lattice_parameters params)
{
    params_ = params;
}

state_lattice_planner::~state_lattice_planner()
{

}

boost::optional<std::pair<nav_msgs::Path,geometry_msgs::Twist>> state_lattice_planner::plan(robotx_msgs::ObstacleMap map, nav_msgs::Odometry odom)
{
    std::pair<nav_msgs::Path,geometry_msgs::Twist> ret;
    for(int i=0; i< params_.num_samples_angular; i++)
    {
        for(int m=0; m<params_.num_samples_linear; m++)
        {
            double linear_acceleration = odom.twist.twist.linear.x;
            double angular_acceleration = odom.twist.twist.angular.z;
        }
    }
    return ret;
}

std::vector<geometry_msgs::Pose2D> state_lattice_planner::generate_path(nav_msgs::Odometry odom, double linear_acceleration, double angular_acceleration)
{
    geometry_msgs::Twist twist;
    twist = odom.twist.twist;
    std::vector<geometry_msgs::Pose2D> path;
    double current_x = 0.0;
    double current_y = 0.0;
    double current_yaw = 0.0;
    geometry_msgs::Pose2D current_pose;
    current_pose.x = 0;
    current_pose.y = 0;
    current_pose.theta = 0;
    for(int i=0; i<params_.num_predictions; i++)
    {
        current_yaw = current_yaw + twist.angular.z * params_.step_duration_;
        current_x = current_x + twist.linear.x * params_.step_duration_ * std::cos(current_yaw);
        current_y = current_y + twist.linear.x * params_.step_duration_ * std::sin(current_yaw);
    }
    return path;
}

double state_lattice_planner::evaluate_function_(robotx_msgs::ObstacleMap map, std::vector<geometry_msgs::Pose2D> path)
{
    return 0;
}