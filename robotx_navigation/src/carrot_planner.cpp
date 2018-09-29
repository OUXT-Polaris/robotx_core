//headers in this package
#include <carrot_planner.h>

carrot_planner::carrot_planner() : _tf_listener(_tf_buffer)
{
    _goal_recieved = false;
    _nh.param<std::string>(ros::this_node::getName()+"/goal_topic", _goal_topic, ros::this_node::getName()+"/goal_pose");
    _nh.param<std::string>(ros::this_node::getName()+"/tolerance_topic", _tolerance_topic, ros::this_node::getName()+"/tolerance");
    _nh.param<std::string>(ros::this_node::getName()+"/map_frame", _map_frame, "map");
    _nh.param<double>(ros::this_node::getName()+"/default_torelancee", _torelance, 1.0);
    _nh.param<double>(ros::this_node::getName()+"/publish_rate", _publish_rate, 10);
    _nh.param<std::string>(ros::this_node::getName()+"/robot_frame", _robot_frame,"base_link");
    _tolerance_sub = _nh.subscribe(_tolerance_topic,1,&carrot_planner::_torelance_callback,this);
    _goal_pose_sub = _nh.subscribe(_goal_topic,1,&carrot_planner::_goal_pose_callback,this);
}

carrot_planner::~carrot_planner()
{

}

void carrot_planner::_torelance_callback(const std_msgs::Float64::ConstPtr msg)
{
    std::unique_lock<std::mutex> lock(_mtx);
    _torelance = msg->data;
    lock.unlock();
    return;
}

void carrot_planner::_goal_pose_callback(geometry_msgs::PoseStamped msg)
{
    std::unique_lock<std::mutex> lock(_mtx);
    geometry_msgs::TransformStamped transform_stamped;
    if(_goal_pose.header.frame_id != _robot_frame)
    {
        try
        {
            transform_stamped = _tf_buffer.lookupTransform(_map_frame, _goal_pose.header.frame_id,ros::Time(0));
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            lock.unlock();
            return;
        }
    }
    tf2::doTransform(msg, _goal_pose, transform_stamped);
    _goal_recieved = true;
    lock.unlock();
    return;
}

void carrot_planner::run()
{
    ros::Rate rate(_publish_rate);
    geometry_msgs::TransformStamped transform_stamped;
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(_mtx);
        if(!_goal_recieved)
        {
            lock.unlock();
            rate.sleep();
            continue;
        }
        if(_goal_pose.header.frame_id != _robot_frame)
        {
            try
            {
                transform_stamped = _tf_buffer.lookupTransform(_robot_frame, _goal_pose.header.frame_id,ros::Time(0));
            }
            catch(tf2::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
                lock.unlock();
                rate.sleep();
                continue;
            }
        }
        lock.unlock();
        rate.sleep();
    }
    return;
}