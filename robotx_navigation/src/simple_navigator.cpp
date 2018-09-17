#include <simple_navigator.h>

simple_navigator::simple_navigator() : 
    _twist_sub(_nh, "/vel_stamped", 1),
    _robot_pose_sub(_nh, "/robot_pose", 1),
    _pose_twist_sync(_sync_policy(10), _robot_pose_sub, _twist_sub)
{
    _nh.param<double>("max_search_radius", _max_search_radius, 20.0);
    _nh.param<double>("max_rotation_speed", _max_rotation_speed, 0.3);
    _nh.param<double>("max_speed", _max_speed, 1.0);
    _nh.param<double>("target_duration" , _target_duration, 10.0);
    _nh.param<double>("publish_rate", _publish_rate, 10.0);
    _euclidean_cluster_sub = _nh.subscribe(ros::this_node::getName()+"/euclidean_cluster", 1, &simple_navigator::_euclidean_cluster_callback, this);
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
        radius = linear_vel * _target_duration;
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
    return;
}

void simple_navigator::_pose_callback(const geometry_msgs::PoseStampedConstPtr robot_pose_msg,const geometry_msgs::TwistStampedConstPtr twist_msg)
{
    _robot_state.update_current_state(*robot_pose_msg,*twist_msg);
    return;
}
