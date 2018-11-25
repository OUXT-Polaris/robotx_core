#include <obstacle_avoid.h>

obstacle_avoid::obstacle_avoid() : tf_listener_(tf_buffer_)
{
    current_state_ = boost::none;
    map_recieved_ = false;
    odom_recieved_ = false;
    twist_cmd_recieved_ = false;
    nh_.param<std::string>(ros::this_node::getName()+"/map_topic", map_topic_, "/obstacle_map");
    nh_.param<std::string>(ros::this_node::getName()+"/cmd_vel_topic", cmd_vel_topic_, ros::this_node::getName()+"/cmd_vel");
    nh_.param<std::string>(ros::this_node::getName()+"/raw_cmd_vel_topic", raw_cmd_vel_topic_, ros::this_node::getName()+"/input_cmd_vel");
    nh_.param<std::string>(ros::this_node::getName()+"/odom_topic", odom_topic_, "/odom");
    nh_.param<std::string>(ros::this_node::getName()+"/target_pose_topic", target_pose_topic_, ros::this_node::getName()+"/target_pose");
    nh_.param<std::string>(ros::this_node::getName()+"/current_state_topic", current_state_topic_, ros::this_node::getName()+"current_state");
    nh_.param<std::string>(ros::this_node::getName()+"/trigger_event_topic", trigger_event_topic_, ros::this_node::getName()+"/trigger_event");
    nh_.param<double>(ros::this_node::getName()+"/search_radius", search_radius_, 3.0);
    nh_.param<double>(ros::this_node::getName()+"/search_angle", search_angle_, 0.3);
    twist_cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    trigger_event_pub_ = nh_.advertise<robotx_msgs::Event>(trigger_event_topic_, 1);
    map_sub_ = nh_.subscribe(map_topic_, 3, &obstacle_avoid::obstacle_map_callback_, this);
    twist_cmd_sub_ = nh_.subscribe(raw_cmd_vel_topic_, 10, &obstacle_avoid::twist_cmd_callback_, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &obstacle_avoid::odom_callback_, this);
    target_pose_sub_ = nh_.subscribe(target_pose_topic_, 10, &obstacle_avoid::target_pose_callback_, this);
}

obstacle_avoid::~obstacle_avoid()
{

}

void obstacle_avoid::current_state_callback_(const robotx_msgs::State::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    *current_state_ = *msg;
    return;
}

void obstacle_avoid::target_pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    target_pose_ = *msg;
    return;
}

void obstacle_avoid::twist_cmd_callback_(const geometry_msgs::Twist::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    twist_cmd_recieved_ = true;
    raw_twist_cmd_ = *msg;
    return;
}

bool obstacle_avoid::obstacle_found_()
{
    for(int i=0; i<map_.points.size(); i++)
    {
        double yaw = std::atan2(map_.points[i].y,map_.points[i].x);
        double radius = std::sqrt(std::pow(map_.points[i].x,2)+std::pow(map_.points[i].y,2));
        if(std::fabs(search_angle_) > std::fabs(yaw) && search_radius_ > radius)
        {
            return true;
        }
    }
    return false;
}

void obstacle_avoid::odom_callback_(const nav_msgs::Odometry::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    odom_recieved_ = true;
    odom_ = *msg;
    geometry_msgs::Twist twist_cmd;
    if(obstacle_found_())
    {
        robotx_msgs::Event event_msg;
        event_msg.trigger_event_name = "obstacle_found";
        event_msg.header.stamp = ros::Time::now();
        trigger_event_pub_.publish(event_msg);
    }
    else
    {
        twist_cmd = raw_twist_cmd_;
        twist_cmd_pub_.publish(twist_cmd);
    }
    return;
}

void obstacle_avoid::obstacle_map_callback_(const robotx_msgs::ObstacleMap::ConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    map_ = *msg;
    map_recieved_ = true;
    return;
}