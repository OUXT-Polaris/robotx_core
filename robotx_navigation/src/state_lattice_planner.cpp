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
            double linear_acceleration = (params_.max_linear_acceleration-params_.min_linear_acceleration)/(params_.num_samples_linear-1)*i
                + params_.min_linear_acceleration;
            double angular_acceleration = (params_.max_angular_acceleration-params_.min_angular_acceleration)/(params_.num_samples_angular-1)*i
                + params_.min_angular_acceleration;
            std::vector<geometry_msgs::Pose2D> path = generate_path(odom, linear_acceleration, angular_acceleration);
            double nearest_obstacle_distance = get_nearest_obstacle_distance_(map, path);
        }
    }
    return ret;
}

std::vector<geometry_msgs::Pose2D> state_lattice_planner::generate_path(nav_msgs::Odometry odom, double linear_acceleration, double angular_acceleration)
{
    geometry_msgs::Twist twist;
    twist = odom.twist.twist;
    std::vector<geometry_msgs::Pose2D> path;
    geometry_msgs::Pose2D current_pose;
    current_pose.x = 0;
    current_pose.y = 0;
    current_pose.theta = 0;
    path.push_back(current_pose);
    for(int i=0; i<params_.num_predictions; i++)
    {
        twist.linear.x = twist.linear.x + linear_acceleration * params_.step_duration_;
        twist.angular.z = twist.angular.z + angular_acceleration * params_.step_duration_;
        if(twist.angular.z > 0 && twist.angular.z > params_.max_angular_velocity)
        {
            twist.angular.z = params_.max_angular_velocity;
        }
        if(twist.angular.z < 0 && twist.angular.z < -1*params_.max_angular_velocity)
        {
            twist.angular.z = -1 * params_.max_angular_velocity;
        }
        if(twist.linear.x > params_.max_linear_velocity)
        {
            twist.linear.x = params_.max_linear_velocity;
        }
        if(twist.linear.x < params_.min_linear_velocity)
        {
            twist.linear.x = params_.min_linear_velocity;
        }
        current_pose.theta = current_pose.theta + twist.angular.z * params_.step_duration_;
        current_pose.x = current_pose.x + twist.linear.x * params_.step_duration_ * std::cos(current_pose.theta);
        current_pose.y = current_pose.y + twist.linear.x * params_.step_duration_ * std::sin(current_pose.theta);
        path.push_back(current_pose);
    }
    return path;
}

double state_lattice_planner::get_nearest_obstacle_distance_(robotx_msgs::ObstacleMap map, std::vector<geometry_msgs::Pose2D> path)
{
    double min_dist = 0;
    for(int i = 0; i < path.size(); i++)
    {
        for(int m = 0; m < map.points.size(); m++)
        {
            if(i == 0 && m ==0)
            {
                min_dist = std::sqrt(std::pow(map.points[m].x - path[i].x,2) + std::pow(map.points[m].y - path[i].y,2)) - map.radius[m];
            }
            else
            {
                double dist = std::sqrt(std::pow(map.points[m].x - path[i].x,2) + std::pow(map.points[m].y - path[i].y,2)) - map.radius[m];
                if(min_dist > dist)
                {
                    min_dist = dist;
                }
            }
        }
    }
    return min_dist;
}