//headers in this package
#include <carrot_planner.h>

carrot_planner::carrot_planner() : _tf_listener(_tf_buffer)
{
    _goal_recieved = false;
    _nh.param<std::string>(ros::this_node::getName()+"/goal_topic", _goal_topic, ros::this_node::getName()+"/goal_pose");
    _nh.param<std::string>(ros::this_node::getName()+"/tolerance_topic", _tolerance_topic, ros::this_node::getName()+"/tolerance");
    _nh.param<std::string>(ros::this_node::getName()+"/map_frame", _map_frame, "map");
    _nh.param<std::string>(ros::this_node::getName()+"/linear_velocity_topic", _linear_velocity_topic, ros::this_node::getName()+"/linear_velocity");
    _nh.param<double>(ros::this_node::getName()+"/default_linear_velocity", _linear_velocity, 0.1);
    _nh.param<double>(ros::this_node::getName()+"/default_torelance", _torelance, 1.0);
    _nh.param<double>(ros::this_node::getName()+"/publish_rate", _publish_rate, 10);
    _nh.param<std::string>(ros::this_node::getName()+"/robot_frame", _robot_frame,"base_link");
    _nh.param<std::string>(ros::this_node::getName()+"/twist_topic", _twist_topic, ros::this_node::getName()+"/twist_cmd");
    _twist_pub = _nh.advertise<geometry_msgs::Twist>(_twist_topic,1);
    _status_pub = _nh.advertise<jsk_rviz_plugins::OverlayText>(ros::this_node::getName()+"/status",1);
    _tolerance_sub = _nh.subscribe(_tolerance_topic,1,&carrot_planner::_torelance_callback,this);
    _goal_pose_sub = _nh.subscribe(_goal_topic,1,&carrot_planner::_goal_pose_callback,this);
    _linear_velocity_sub = _nh.subscribe(_linear_velocity_topic,1,&carrot_planner::_linear_velocity_callback,this);
}

carrot_planner::~carrot_planner()
{

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
            ROS_ERROR("%s",ex.what());
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
    geometry_msgs::TransformStamped transform_stamped;
    jsk_rviz_plugins::OverlayText text;
    text.action = text.ADD;
    text.width = 320;
    text.height = 50;
    text.left = 0;
    text.top = 0;
    text.bg_color.r = 0;
    text.bg_color.g = 0;
    text.bg_color.b = 0;
    text.bg_color.a = 0.5;
    text.line_width = 20;
    text.text_size = 20;
    text.fg_color.r = 0;
    text.fg_color.g = 1.0;
    text.fg_color.b = 1.0;
    text.fg_color.a = 1.0;
    while(ros::ok())
    {
        std::unique_lock<std::mutex> lock(_mtx);
        geometry_msgs::Twist twist_cmd;
        if(_goal_recieved == false)
        {
            text.text = "no goal pose";
            _status_pub.publish(text);
            lock.unlock();
            rate.sleep();
            continue;
        }
        if(_map_frame != _robot_frame)
        {
            try
            {
                transform_stamped = _tf_buffer.lookupTransform(_robot_frame, _map_frame,ros::Time(0));
            }
            catch(tf2::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                text.text = "failed to transform goal pose";
                _status_pub.publish(text);
                lock.unlock();
                rate.sleep();
                continue;
            }
        }
        geometry_msgs::PoseStamped transformed_pose;
        tf2::doTransform(_goal_pose, transformed_pose, transform_stamped);
        double radius = std::sqrt(std::pow(transformed_pose.pose.position.x,2)+std::pow(transformed_pose.pose.position.y,2));
        if(radius < _torelance)
        {
            tf::Quaternion q(transformed_pose.pose.orientation.x, transformed_pose.pose.orientation.y, transformed_pose.pose.orientation.z, transformed_pose.pose.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            if(yaw < 0)
            {
                text.text = "align to goal pose";
                _status_pub.publish(text);
                twist_cmd.linear.x = 0;
                twist_cmd.linear.y = 0;
                twist_cmd.angular.z = 0.1;
            }
            else
            {
                text.text = "align to goal pose";
                _status_pub.publish(text);
                twist_cmd.linear.x = 0;
                twist_cmd.linear.y = 0;
                twist_cmd.angular.z = -0.1;
            }
        }
        else
        {
            double phi = std::atan2(transformed_pose.pose.position.y,transformed_pose.pose.position.x);
            if(-0.5*M_PI < phi && phi < 0.5*M_PI)
            {
                double dt = std::fabs((radius*phi)/(_linear_velocity*std::sin(phi)));
                if(radius<3.0)
                {
                    text.text = "approach to goal pose";
                    _status_pub.publish(text);
                    twist_cmd.linear.x = 0.5;
                }
                else
                {
                    text.text = "move to goal pose";
                    _status_pub.publish(text);
                    twist_cmd.linear.x = _linear_velocity;
                }
                twist_cmd.linear.y = 0;
                twist_cmd.angular.z = phi/dt;
            }
            else
            {
                double dt = std::fabs((radius*phi)/(_linear_velocity*std::sin(phi)));
                if(radius<3.0)
                {
                    text.text = "approach to goal pose";
                    _status_pub.publish(text);
                    twist_cmd.linear.x = -0.5;
                }
                else
                {
                    text.text = "move to goal pose";
                    _status_pub.publish(text);
                    twist_cmd.linear.x = -1*_linear_velocity;
                }
                twist_cmd.linear.y = 0;
                twist_cmd.angular.z = -phi/dt;
            }
        }
        _twist_pub.publish(twist_cmd);
        lock.unlock();
        rate.sleep();
    }
    return;
}