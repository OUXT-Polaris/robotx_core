//headers in this package
#include <carrot_planner.h>

carrot_planner::carrot_planner() : _tf_listener(_tf_buffer)
{
    _goal_recieved = false;
    _nh.param<std::string>(ros::this_node::getName()+"/goal_topic", _goal_topic, ros::this_node::getName()+"/goal_pose");
    _nh.param<std::string>(ros::this_node::getName()+"/tolerance_topic", _tolerance_topic, ros::this_node::getName()+"/tolerance");
    _nh.param<std::string>(ros::this_node::getName()+"/angular_tolerance_topic", _angular_tolerance_topic, ros::this_node::getName()+"/angular_tolerance");
    _nh.param<std::string>(ros::this_node::getName()+"/map_frame", _map_frame, "map");
    _nh.param<std::string>(ros::this_node::getName()+"/linear_velocity_topic", _linear_velocity_topic, ros::this_node::getName()+"/linear_velocity");
    _nh.param<double>(ros::this_node::getName()+"/default_linear_velocity", _linear_velocity, 0.1);
    _nh.param<double>(ros::this_node::getName()+"/default_torelance", _torelance, 1.0);
    _nh.param<double>(ros::this_node::getName()+"/default_angular_tolerance", _angular_tolerance, 1.57);
    _nh.param<double>(ros::this_node::getName()+"/publish_rate", _publish_rate, 10);
    _nh.param<std::string>(ros::this_node::getName()+"/robot_frame", _robot_frame,"base_link");
    _nh.param<std::string>(ros::this_node::getName()+"/twist_topic", _twist_topic, ros::this_node::getName()+"/twist_cmd");
    _twist_pub = _nh.advertise<geometry_msgs::Twist>(_twist_topic,1);
    _trigger_event_pub = _nh.advertise<robotx_msgs::Event>("/robotx_state_machine_node/navigation_state_machine/trigger_event",1);
    _tolerance_sub = _nh.subscribe(_tolerance_topic,1,&carrot_planner::_torelance_callback,this);
    _goal_pose_sub = _nh.subscribe(_goal_topic,1,&carrot_planner::_goal_pose_callback,this);
    _linear_velocity_sub = _nh.subscribe(_linear_velocity_topic,1,&carrot_planner::_linear_velocity_callback,this);
}

carrot_planner::~carrot_planner()
{

}

void carrot_planner::_current_state_callback(const robotx_msgs::State::ConstPtr msg)
{
    std::unique_lock<std::mutex> lock(_mtx);
    _current_state = *msg;
    lock.unlock();
    return;
}

void carrot_planner::_linear_velocity_callback(const std_msgs::Float64::ConstPtr msg)
{
    std::unique_lock<std::mutex> lock(_mtx);
    _linear_velocity = msg->data;
    lock.unlock();
    return;
}

void carrot_planner::_torelance_callback(const std_msgs::Float64::ConstPtr msg)
{
    std::unique_lock<std::mutex> lock(_mtx);
    _torelance = msg->data;
    lock.unlock();
    return;
}

void carrot_planner::_angular_torelance_callback(const std_msgs::Float64::ConstPtr msg)
{
    std::unique_lock<std::mutex> lock(_mtx);
    _angular_tolerance = msg->data;
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
            transform_stamped = _tf_buffer.lookupTransform(_map_frame, msg.header.frame_id,ros::Time(0));
            tf2::doTransform(msg, _goal_pose, transform_stamped);
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            lock.unlock();
            return;
        }
    }
    else
    {
        _goal_pose = msg;
    }
    _goal_recieved = true;
    lock.unlock();
    return;
}

void carrot_planner::run()
{
    boost::thread cmd_publish_thread(boost::bind(&carrot_planner::_publish_twist_cmd,this));
    return;
}

void carrot_planner::_publish_twist_cmd()
{
    ros::Rate rate(_publish_rate);
    while(ros::ok())
    {
        rate.sleep();
    }
    return;
}