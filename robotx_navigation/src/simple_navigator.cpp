#include <simple_navigator.h>

simple_navigator::simple_navigator() : 
    _twist_sub(_nh, "/vel_stamped", 1),
    _robot_pose_sub(_nh, "/robot_pose", 1),
    _pose_twist_sync(_sync_policy(10), _robot_pose_sub, _twist_sub)
{
    _nh.param<double>(ros::this_node::getName()+"/max_search_radius", _max_search_radius, 20.0);
    _nh.param<double>(ros::this_node::getName()+"/max_rotation_speed", _max_rotation_speed, 0.3);
    _nh.param<double>(ros::this_node::getName()+"/max_speed", _max_speed, 1.0);
    _nh.param<double>(ros::this_node::getName()+"/target_duration" , _target_duration, 10.0);
    _nh.param<double>(ros::this_node::getName()+"/publish_rate", _publish_rate, 10.0);
    _nh.param<double>(ros::this_node::getName()+"/buffer_length", _buffer_length, 10.0);
    _nh.param<double>(ros::this_node::getName()+"/max_cluster_length", _max_cluster_length, 5.0);
    _nh.param<double>(ros::this_node::getName()+"/min_cluster_length", _min_cluster_length, 0.3);
    _nh.param<std::string>(ros::this_node::getName()+"/map_frame", _map_frame, "map");
    _nh.param<std::string>(ros::this_node::getName()+"/robot_frame", _robot_frame, "base_link");
    _marker_pub = _nh.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName()+"/marker", 1);
    _twist_cmd_pub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    _euclidean_cluster_sub = _nh.subscribe(ros::this_node::getName()+"/euclidean_cluster", 1, &simple_navigator::_euclidean_cluster_callback, this);
    _buffer = boost::make_shared<euclidean_cluster_buffer>(_buffer_length, _map_frame);
    _goal_pose_sub = _nh.subscribe("/move_base_simple/goal", 1, &simple_navigator::_goal_pose_callback, this);
    _pose_twist_sync.registerCallback(boost::bind(&simple_navigator::_pose_callback, this, _1, _2));
}

simple_navigator::~simple_navigator()
{

}

void simple_navigator::run()
{
    boost::thread pub_thread(boost::bind(&simple_navigator::_publish_twist_cmd, this));
}

void simple_navigator::_publish_marker(double search_radius)
{
    visualization_msgs::MarkerArray msg;
    //ros::Time now = ros::Time::now();
    visualization_msgs::Marker circle_marker;
    circle_marker.header.stamp = ros::Time::now();
    circle_marker.header.frame_id = "base_footprint";
    circle_marker.id = 0;
    circle_marker.type = circle_marker.CYLINDER;
    circle_marker.action = circle_marker.ADD;
    circle_marker.pose.position.x = 0;
    circle_marker.pose.position.y = 0;
    circle_marker.pose.position.z = 0;
    circle_marker.pose.orientation.x = 0;
    circle_marker.pose.orientation.y = 0;
    circle_marker.pose.orientation.z = 0;
    circle_marker.pose.orientation.w = 1;
    circle_marker.scale.x = search_radius*2;
    circle_marker.scale.y = search_radius*2;
    circle_marker.scale.z = 0.3;
    circle_marker.color.r = 0;
    circle_marker.color.g = 1;
    circle_marker.color.b = 0;
    circle_marker.color.a = 0.5;
    msg.markers.push_back(circle_marker);
    _marker_pub.publish(msg);
    return;
}

void simple_navigator::_publish_twist_cmd()
{
    ros::Rate rate(_publish_rate);
    while(ros::ok())
    {
        boost::optional<robot_state_info> state_info = _robot_state.get_robot_state();
        if(!state_info)
        {
            rate.sleep();
            continue;
        }
        double search_radius = _get_search_radius(*state_info);
        _publish_marker(search_radius);
        rate.sleep();
    }
    return;
}

double simple_navigator::_get_distance(double r, double theta, cluster_data euclidean_cluster)
{
    double distance = 0;
    double phi = std::atan2(euclidean_cluster.point.point.y,euclidean_cluster.point.point.x);
    if(0 < phi && phi < theta)
    {
        double l = std::sqrt(std::pow(euclidean_cluster.point.point.x-r,2)+std::pow(euclidean_cluster.point.point.y,2));
        distance = l - r;
    }
    return distance;
}

double simple_navigator::_get_search_radius(robot_state_info state_info)
{
    double radius;
    double linear_vel = state_info.current_twist->twist.linear.x;
    //double rot_vel = state_info.current_twist->twist.angular.z;
    radius = std::fabs(linear_vel) * _target_duration;
    if(state_info.current_pose && state_info.goal_pose)
    {
        double target_range = 
            std::sqrt(std::pow(state_info.current_pose->pose.position.x - state_info.goal_pose->pose.position.x,2) + 
                std::pow(state_info.current_pose->pose.position.y - state_info.goal_pose->pose.position.y,2));
        if(radius > target_range)
        {
            radius = target_range;
        }
        //return radius;
    }
    if(radius > _max_search_radius)
    {
        radius = _max_search_radius;
    }
    return radius;
}

void simple_navigator::_goal_pose_callback(const geometry_msgs::PoseStampedConstPtr msg)
{
    _robot_state.update_goal_pose(*msg);
    return;
}

void simple_navigator::_euclidean_cluster_callback(const jsk_recognition_msgs::BoundingBoxArrayConstPtr msg)
{
    boost::optional<robot_state_info> state_info = _robot_state.get_robot_state();
    if(!state_info)
    {
        return;
    }
    boost::optional<double> search_radius = _get_search_radius(*state_info);
    if(!search_radius)
    {
        return;
    }
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

void simple_navigator::_pose_callback(const geometry_msgs::PoseStampedConstPtr robot_pose_msg,const geometry_msgs::TwistStampedConstPtr twist_msg)
{
    _robot_state.update_current_state(*robot_pose_msg,*twist_msg);
    return;
}
