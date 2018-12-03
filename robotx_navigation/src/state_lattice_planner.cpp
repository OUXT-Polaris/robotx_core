#include <state_lattice_planner.h>

state_lattice_planner::state_lattice_planner()
{
    params_.max_angular_acceleration = params_.DEFAULT_MAX_ANGULAR_ACCERALATION;
    params_.min_angular_acceleration = params_.DEFAULT_MIN_ANGULAR_ACCERALATION;
    params_.max_linear_acceleration  = params_.DEFAULT_MAX_LINEAR_ACCERALATION;
    params_.min_linear_acceleration  = params_.DEFAULT_MIN_LINEAR_ACCERALATION;
    params_.max_linear_velocity      = params_.DEFAULT_MAX_LINEAR_VELOCITY;
    params_.min_linear_velocity      = params_.DEFAULT_MIN_LINEAR_VELOCITY;
    params_.max_angular_velocity     = params_.DEFAULT_MAX_ANGULAR_VELOCITY;
    params_.step_duration            = params_.DEFAULT_STEP_DURATION;
    params_.num_predictions          = params_.DEFAULT_NUM_PREDICTIONS;
    params_.num_samples_angular      = params_.DEFAULT_NUM_SAMPLES_ANGULAR;
    params_.num_samples_linear       = params_.DEFAULT_NUM_SAMPLES_LINEAR;
}

state_lattice_planner::state_lattice_planner(robotx_msgs::ObstacleAvoidConfigure params)
{
    params_ = params;
}

state_lattice_planner::~state_lattice_planner()
{

}

void state_lattice_planner::update_params(robotx_msgs::ObstacleAvoidConfigure params)
{
    params_ = params;
    return;
}

boost::optional<geometry_msgs::Twist> state_lattice_planner::plan(robotx_msgs::ObstacleMap map, nav_msgs::Odometry odom, geometry_msgs::Pose2D target_pose)
{
    std::vector<double> evaluate_values(params_.num_samples_angular*params_.num_samples_linear);
    std::vector<geometry_msgs::Twist> twists(params_.num_samples_angular*params_.num_samples_linear);
    for(int i=0; i< params_.num_samples_angular; i++)
    {
        for(int m=0; m<params_.num_samples_linear; m++)
        {
            double linear_acceleration = (params_.max_linear_acceleration-params_.min_linear_acceleration)/(params_.num_samples_linear-1)*m
                + params_.min_linear_acceleration;
            double angular_acceleration = (params_.max_angular_acceleration-params_.min_angular_acceleration)/(params_.num_samples_angular-1)*i
                + params_.min_angular_acceleration;
            std::vector<geometry_msgs::Pose2D> path = generate_path(odom, linear_acceleration, angular_acceleration);
            double nearest_obstacle_distance = get_nearest_obstacle_distance_(map, path);
            double evaluate_value = evaluate_function_(nearest_obstacle_distance, path[path.size()-1], target_pose);
            evaluate_values[i*params_.num_samples_linear+m] = evaluate_value;
            geometry_msgs::Twist twist_cmd = odom.twist.twist;
            twist_cmd.linear.x = twist_cmd.linear.x + linear_acceleration;
            twist_cmd.angular.z = twist_cmd.angular.z + angular_acceleration;
            twists[i*params_.num_samples_linear+m] = twist_cmd;
        }
    }
    int index = *std::max_element(evaluate_values.begin(), evaluate_values.end());
    if(evaluate_values[index] == 0)
    {
        return boost::none;
    }
    return twists[index];
}

double state_lattice_planner::evaluate_function_(double nearest_obstacle_distance, geometry_msgs::Pose2D end_pose, geometry_msgs::Pose2D target_pose)
{
    double ret;
    if(nearest_obstacle_distance < 0)
    {
        return 0;
    }
    if(std::pow(end_pose.x - target_pose.x,2) + std::pow(end_pose.y - target_pose.y,2) < 0.0001)
    {
        return 1000;
    }
    ret = nearest_obstacle_distance/std::sqrt(std::pow(end_pose.x - target_pose.x,2) + std::pow(end_pose.y - target_pose.y,2));
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
        twist.linear.x = twist.linear.x + linear_acceleration * params_.step_duration;
        twist.angular.z = twist.angular.z + angular_acceleration * params_.step_duration;
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
        current_pose.theta = current_pose.theta + twist.angular.z * params_.step_duration;
        current_pose.x = current_pose.x + twist.linear.x * params_.step_duration * std::cos(current_pose.theta);
        current_pose.y = current_pose.y + twist.linear.x * params_.step_duration * std::sin(current_pose.theta);
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