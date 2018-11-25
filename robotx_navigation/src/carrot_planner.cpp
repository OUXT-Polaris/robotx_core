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
    _current_stete_sub = _nh.subscribe("/robotx_state_machine_node/navigation_state_machine/current_state",1,&carrot_planner::_current_state_callback,this);
    _robot_pose_sub = _nh.subscribe("/robot_pose",1,&carrot_planner::_robot_pose_callback,this);
    _tolerance_sub = _nh.subscribe(_tolerance_topic,1,&carrot_planner::_torelance_callback,this);
    _goal_pose_sub = _nh.subscribe(_goal_topic,1,&carrot_planner::_goal_pose_callback,this);
    _linear_velocity_sub = _nh.subscribe(_linear_velocity_topic,1,&carrot_planner::_linear_velocity_callback,this);
}

carrot_planner::~carrot_planner()
{

}

void carrot_planner::_robot_pose_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    std::unique_lock<std::mutex> lock(_mtx);
    geometry_msgs::PoseStamped robot_pose;
    if(msg->header.frame_id != _map_frame)
    {
        geometry_msgs::TransformStamped transform_stamped;
        try
        {
            transform_stamped = _tf_buffer.lookupTransform(_map_frame, msg->header.frame_id, ros::Time(0));
            tf2::doTransform(*msg, robot_pose, transform_stamped);
        }
        catch(tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            lock.unlock();
            return;
        }
    }
    tf::Quaternion quat(robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z,robot_pose.pose.orientation.w);
    double r,p,y;
    tf::Matrix3x3(quat).getRPY(r,p,y);
    _robot_pose_2d.x = robot_pose.pose.position.x;
    _robot_pose_2d.y = robot_pose.pose.position.y;
    _robot_pose_2d.theta = y;
    lock.unlock();
    return;
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
    if(_goal_pose.header.frame_id != _map_frame)
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
    _goal_pose_2d.x = _goal_pose.pose.position.x;
    _goal_pose_2d.y = _goal_pose.pose.position.y;
    tf::Quaternion quat(_goal_pose.pose.orientation.x,_goal_pose.pose.orientation.y,_goal_pose.pose.orientation.z,_goal_pose.pose.orientation.w);
    double r,p,y;
    tf::Matrix3x3(quat).getRPY(r,p,y);
    _goal_pose_2d.theta = y;
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
        if(!_current_state)
        {
            rate.sleep();
            continue;
        }
        if(_current_state->current_state == "heading_to_next_waypoint")
        {
            if(std::fabs(_get_diff_yaw()) < 0.1)
            {
                if(_get_diff_yaw() > 0)
                {
                    geometry_msgs::Twist twist_cmd;
                    twist_cmd.angular.z = 0.1;
                    _twist_pub.publish(twist_cmd);
                    rate.sleep();
                    continue;
                }
                else
                {
                    geometry_msgs::Twist twist_cmd;
                    twist_cmd.angular.z = -0.1;
                    _twist_pub.publish(twist_cmd);
                    rate.sleep();
                    continue;
                }
            }
        }
        if(_current_state->current_state == "moving_to_next_waypoint")
        {
        }
        if(_current_state->current_state == "align_to_next_waypoint")
        {
        }
        rate.sleep();
    }
    return;
}

double carrot_planner::_get_diff_yaw()
{
    double ret;
    if(_robot_pose_2d.theta > _goal_pose_2d.theta)
    {
        double a1 = _robot_pose_2d.theta-_goal_pose_2d.theta;
        double a2 = 2*M_PI - a1;
        if(a1 < a2)
        {
            return a1;
        }
        else
        {
            return -1 * a2;
        }
    }
    else
    {
        double a1 = _goal_pose_2d.theta - _robot_pose_2d.theta;
        double a2 = 2*M_PI - a1;
        if(a1 < a2)
        {
            return a1;
        }
        else
        {
            return -1 * a2;
        }
    }
    return ret;
}