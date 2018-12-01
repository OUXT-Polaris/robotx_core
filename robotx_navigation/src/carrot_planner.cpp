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
        double diff_yaw_to_target = _get_diff_yaw_to_target();
        if(_current_state->current_state == "heading_to_next_waypoint")
        {
            if(std::sqrt(std::pow(_goal_pose_2d.x-_robot_pose_2d.x,2)-std::pow(_goal_pose_2d.y-_robot_pose_2d.y,2)) < _torelance)
            {
                geometry_msgs::Twist twist_cmd;
                _twist_pub.publish(twist_cmd);
                robotx_msgs::Event event_msg;
                event_msg.trigger_event_name = "moved_to_next_target";
                _trigger_event_pub.publish(event_msg);
                rate.sleep();
                continue;
            }
            if(std::fabs(diff_yaw_to_target) > 0.05)
            {
                if(diff_yaw_to_target > 0)
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
            else
            {
                geometry_msgs::Twist twist_cmd;
                _twist_pub.publish(twist_cmd);
                robotx_msgs::Event event_msg;
                event_msg.trigger_event_name = "head_to_next_target";
                _trigger_event_pub.publish(event_msg);
                rate.sleep();
                continue;
            }
        }
        if(_current_state->current_state == "moving_to_next_waypoint")
        {
            if(std::sqrt(std::pow(_goal_pose_2d.x-_robot_pose_2d.x,2)-std::pow(_goal_pose_2d.y-_robot_pose_2d.y,2)) < _torelance)
            {
                geometry_msgs::Twist twist_cmd;
                _twist_pub.publish(twist_cmd);
                robotx_msgs::Event event_msg;
                event_msg.trigger_event_name = "moved_to_next_target";
                _trigger_event_pub.publish(event_msg);
                rate.sleep();
                continue;
            }
            else if(diff_yaw_to_target && std::fabs(diff_yaw_to_target > 0.1))
            {
                geometry_msgs::Twist twist_cmd;
                _twist_pub.publish(twist_cmd);
                robotx_msgs::Event event_msg;
                event_msg.trigger_event_name = "heading_slipped";
                _trigger_event_pub.publish(event_msg);
                rate.sleep();
                continue;
            }
            else
            {
                geometry_msgs::Twist twist_cmd;
                twist_cmd.linear.x = _linear_velocity;
                _twist_pub.publish(twist_cmd);
                rate.sleep();
                continue;
            }
        }
        if(_current_state->current_state == "align_to_next_waypoint")
        {
            double diff_yaw = _get_diff_yaw();
            if(diff_yaw > 0.1)
            {
                geometry_msgs::Twist twist_cmd;
                twist_cmd.angular.z = 0.1;
                _twist_pub.publish(twist_cmd);
                rate.sleep();
                continue;
            }
            if(diff_yaw < -0.1)
            {
                geometry_msgs::Twist twist_cmd;
                twist_cmd.angular.z = -0.1;
                _twist_pub.publish(twist_cmd);
                rate.sleep();
                continue;
            }
            if(std::fabs(diff_yaw) <= 0.1)
            {
                geometry_msgs::Twist twist_cmd;
                _twist_pub.publish(twist_cmd);
                robotx_msgs::Event event_msg;
                event_msg.trigger_event_name = "waypoint_reached";
                _trigger_event_pub.publish(event_msg);
                rate.sleep();
                continue;
            }
        }
        rate.sleep();
    }
    return;
}

double carrot_planner::get_diff_angle_(double from,double to)
{
    double ans = 0;
    double inner_prod = std::cos(from)*std::cos(to)+std::sin(from)*std::sin(to);
    double theta = std::acos(inner_prod);
    double outer_prod = std::cos(from)*std::sin(to)-std::sin(from)*std::cos(to);
    if(outer_prod > 0)
    {
        ans = theta;
    }
    else
    {
        ans = -1 * theta;
    }
    return ans;
}

double carrot_planner::_get_diff_yaw()
{
    return get_diff_angle_(_robot_pose_2d.theta,_goal_pose_2d.theta);
}

double carrot_planner::_get_diff_yaw_to_target()
{
    double yaw_to_target = std::atan2(_goal_pose_2d.y-_robot_pose_2d.y, _goal_pose_2d.x-_robot_pose_2d.x);// - M_PI/2;
    return get_diff_angle_(_robot_pose_2d.theta,yaw_to_target);
}