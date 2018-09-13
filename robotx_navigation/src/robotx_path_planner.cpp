#include <robotx_path_planner.h>

robotx_path_planner::robotx_path_planner() : _tf_listener(_tf_buffer)
{
    double buffer_length;
    _nh.param<double>(ros::this_node::getName()+"/buffer_length", buffer_length, 1.0);
    _nh.param<double>(ros::this_node::getName()+"/max_cluster_length", _max_cluster_length, 5.0);
    _nh.param<double>(ros::this_node::getName()+"/min_cluster_length", _min_cluster_length, 0.3);
    _nh.param<double>(ros::this_node::getName()+"/inflation_radius", _inflation_radius, 0.5);
    _nh.param<std::string>(ros::this_node::getName()+"/map_frame", _map_frame, "world");
    _marker_pub = _nh.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName()+"/marker", 1);
    _buffer = boost::make_shared<euclidean_cluster_buffer>(buffer_length, _map_frame);
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
    //ROS_ERROR_STREAM(clusters.size());
    visualization_msgs::MarkerArray marker_array = _generate_markers(clusters);
    _marker_pub.publish(marker_array);
    return;
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