#include <simple_navigator.h>

simple_navigator::simple_navigator() : 
    _twist_sub(_nh, "/vel_stamped", 1),
    _robot_pose_sub(_nh, "/robot_pose", 1),
    _pose_twist_sync(_sync_policy(10), _robot_pose_sub, _twist_sub)
{
    _nh.param<double>("min_search_radius", _min_search_radius, 10.0);
    _nh.param<double>("max_search_radius", _max_search_radius, 40.0);
    _nh.param<double>("max_rotation_speed", _max_rotation_speed, 0.3);
    _nh.param<double>("max_speed", _max_speed, 1.0);
    _nh.param<double>("publish_rate", _publish_rate, 10);
    _euclidean_cluster_sub = _nh.subscribe(ros::this_node::getName()+"/euclidean_cluster", 1, &simple_navigator::_euclidean_cluster_callback, this);
    _goal_pose_sub = _nh.subscribe("/move_base_simple/goal", 1, &simple_navigator::_goal_pose_callback, this);
    //_robot_pose_sub = _nh.subscribe("/robot_pose", 1, &simple_navigator::_robot_pose_callback, this);
}

simple_navigator::~simple_navigator()
{

}

void simple_navigator::_goal_pose_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
    return;
}

/*
void simple_navigator::_robot_pose_callback(const geometry_msgs::PoseStampedConstPtr msg)
{

}
*/

void simple_navigator::_euclidean_cluster_callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr msg)
{
    return;
}
