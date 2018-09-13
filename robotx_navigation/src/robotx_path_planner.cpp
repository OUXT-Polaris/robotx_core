#include <robotx_path_planner.h>

robotx_path_planner::robotx_path_planner()
{
    _euclidean_cluster_sub = _nh.subscribe(ros::this_node::getName()+"/euclidean_cluster", 1, &robotx_path_planner::_euclidean_cluster_callback, this);
}

robotx_path_planner::~robotx_path_planner()
{
    
}

void robotx_path_planner::_euclidean_cluster_callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr msg)
{
    return;
}