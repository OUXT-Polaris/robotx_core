#include <robotx_path_planner.h>

robotx_path_planner::robotx_path_planner() : _tf_listener(_tf_buffer)
{
    _goal_recieved = false;
    double buffer_length;
    _nh.param<double>(ros::this_node::getName()+"/buffer_length", buffer_length, 1.0);
    _nh.param<double>(ros::this_node::getName()+"/max_cluster_length", _max_cluster_length, 5.0);
    _nh.param<double>(ros::this_node::getName()+"/min_cluster_length", _min_cluster_length, 0.3);
    _nh.param<double>(ros::this_node::getName()+"/inflation_radius", _inflation_radius, 0.5);
    _nh.param<double>(ros::this_node::getName()+"/robot_radius", _robot_radius, 3.0);
    _nh.param<std::string>(ros::this_node::getName()+"/map_frame", _map_frame, "world");
    _marker_pub = _nh.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName()+"/marker", 1);
    _path_pub = _nh.advertise<robotx_msgs::SplinePath>(ros::this_node::getName()+"/path", 1);
    _buffer = boost::make_shared<euclidean_cluster_buffer>(buffer_length, _map_frame);
    _goal_pose_sub = _nh.subscribe("/move_base_simple/goal", 1, &robotx_path_planner::_goal_pose_callback, this);
    _robot_pose_sub = _nh.subscribe("/robot_pose", 1, &robotx_path_planner::_pose_callback, this);
    _euclidean_cluster_sub = _nh.subscribe(ros::this_node::getName()+"/euclidean_cluster", 1, &robotx_path_planner::_euclidean_cluster_callback, this);
}

robotx_path_planner::~robotx_path_planner()
{
    
}

void robotx_path_planner::_goal_pose_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
    geometry_msgs::TransformStamped transform_stamped;
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose;
    if(_map_frame != msg->header.frame_id)
    {
        try
        {
            transform_stamped = _tf_buffer.lookupTransform(_map_frame, msg->header.frame_id, msg->header.stamp);
            tf2::doTransform(pose, pose, transform_stamped);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }
    }
    _mtx.lock();
    _goal_pose = pose;
    _goal_recieved = true;
    _mtx.unlock();
    return;
}

void robotx_path_planner::_pose_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
    std::vector<cluster_data> clusters = _buffer->get_cluster_data();
    geometry_msgs::TransformStamped transform_stamped;
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose;
    if(_map_frame != msg->header.frame_id)
    {
        try
        {
            transform_stamped = _tf_buffer.lookupTransform(_map_frame, msg->header.frame_id, msg->header.stamp);
            tf2::doTransform(pose, pose, transform_stamped);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }
    }
    if(_goal_recieved == false)
    {
        return;
    }
    _mtx.lock();
    std::vector<cluster_data> filtered_clusters = _filter_clusters(clusters, pose.pose.position, _goal_pose.pose.position);
    robotx_msgs::SplinePath path_msg;
    path_msg.header.frame_id = _map_frame;
    path_msg.header.stamp = ros::Time::now();
    if(filtered_clusters.size() == 0)
    {
        path_msg.waypoints.push_back(pose.pose.position);
        path_msg.waypoints.push_back( _goal_pose.pose.position);
        path_msg.waypoints[0].z = 0;
        path_msg.waypoints[1].z = 0;
    }
    visualization_msgs::MarkerArray marker_array = _generate_markers(clusters);
    _marker_pub.publish(marker_array);
    _path_pub.publish(path_msg);
    _mtx.unlock();
    return;
}

std::vector<cluster_data> robotx_path_planner::_filter_clusters(std::vector<cluster_data> clusters, geometry_msgs::Point start_point, geometry_msgs::Point end_point)
{
    std::vector<cluster_data> ret;
    for(int i=0; i<clusters.size(); i++)
    {
        if(_get_range(clusters[i].point.point,start_point,end_point) < (clusters[i].radius+_robot_radius))
        {
            ret.push_back(clusters[i]);
        }
    }
    return ret;
}

// See also https://qiita.com/yellow_73/items/bcd4e150e7caa0210ee6
double robotx_path_planner::_get_range(geometry_msgs::Point circle_center, geometry_msgs::Point start_point, geometry_msgs::Point end_point)
{
    double a = end_point.x - start_point.x;
    double b = end_point.y - start_point.y;
    double a2 = a*a;
    double b2 = b*b;
    double r2 = a2 + b2;
    double tt = -(a*(start_point.x-circle_center.x)+b*(start_point.y-circle_center.y));
    if(tt < 0)
    {
        return (start_point.x-circle_center.x)*(start_point.x-circle_center.x)+(start_point.y-circle_center.y)*(start_point.y-circle_center.y);
    }
    if(tt > r2)
    {
        return (end_point.x-circle_center.x)*(end_point.x-circle_center.x)+(end_point.y-circle_center.y)*(end_point.y-circle_center.y);
    }
    double f1 = a*(start_point.y-circle_center.y) - b*(start_point.x-circle_center.x);
    return (f1*f1)/r2;
}

visualization_msgs::MarkerArray robotx_path_planner::_generate_markers(std::vector<cluster_data> data)
{
    visualization_msgs::MarkerArray ret;
    for(int i=0; i<data.size() ; i++)
    {
        visualization_msgs::Marker marker_msg;
        marker_msg.header.stamp = ros::Time::now();
        marker_msg.header.frame_id = _map_frame;
        marker_msg.type = marker_msg.CYLINDER;
        marker_msg.action = marker_msg.ADD;
        marker_msg.pose.position = data[i].point.point;
        marker_msg.pose.orientation.x = 0;
        marker_msg.pose.orientation.y = 0;
        marker_msg.pose.orientation.z = 0;
        marker_msg.pose.orientation.w = 1;
        marker_msg.scale.x = data[i].radius;
        marker_msg.scale.y = data[i].radius;
        marker_msg.scale.z = 1;
        marker_msg.color.r = 0;
        marker_msg.color.g = 1;
        marker_msg.color.b = 1;
        marker_msg.color.a = 1;
        marker_msg.frame_locked = true;
        marker_msg.id = i;
        ret.markers.push_back(marker_msg);
    }
    return ret;
}

void robotx_path_planner::_euclidean_cluster_callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr msg)
{
    for(int i=0; i<msg->boxes.size(); i++)
    {
        jsk_recognition_msgs::BoundingBox bbox = msg->boxes[i];
        if(bbox.dimensions.x > bbox.dimensions.y)
        {
            if((bbox.dimensions.x < _max_cluster_length) && (bbox.dimensions.y > _min_cluster_length))
            {
                geometry_msgs::PointStamped bbox_point;
                double bbox_size;
                bbox_point.point = bbox.pose.position;
                bbox_point.header = msg->header;
                bbox_size = bbox.dimensions.x;
                cluster_data data(bbox_point, bbox_size);
                _buffer->add_cluster_data(data);
            }
        }
        else
        {
            if((bbox.dimensions.y < _max_cluster_length) && (bbox.dimensions.x > _min_cluster_length))
            {
                geometry_msgs::PointStamped bbox_point;
                double bbox_size;
                bbox_point.point = bbox.pose.position;
                bbox_point.header = msg->header;
                bbox_size = bbox.dimensions.y;
                cluster_data data(bbox_point, bbox_size);
                _buffer->add_cluster_data(data);
            }            
        }
    }
    return;
}