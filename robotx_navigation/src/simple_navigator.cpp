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
    _euclidean_cluster_sub = _nh.subscribe(ros::this_node::getName()+"/euclidean_cluster", 1, &simple_navigator::_euclidean_cluster_callback, this);
    _buffer = boost::make_shared<euclidean_cluster_buffer>(_buffer_length, _map_frame);
    _goal_pose_sub = _nh.subscribe("/move_base_simple/goal", 1, &simple_navigator::_goal_pose_callback, this);
    _pose_twist_sync.registerCallback(boost::bind(&simple_navigator::_pose_callback, this, _1, _2));
}

simple_navigator::~simple_navigator()
{

}

boost::optional<double> simple_navigator::_get_search_radius(robot_state_info state_info)
{
    if(state_info.current_twist)
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
                radius = target_range;
            //return radius;
        }
        if(radius > _max_search_radius)
        {
            radius = _max_search_radius;
            return radius;
        }
    }
    return boost::none;
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
