#include <waypoint_logger.h>

waypoint_logger::waypoint_logger() : tf_listener_(tf_buffer_)
{
    last_waypoints_ = boost::none;
    nh_.param<std::string>(ros::this_node::getName()+"/joy_topic", joy_topic_, "/joy");
    nh_.param<std::string>(ros::this_node::getName()+"/pose_topic", pose_topic_, "/robot_pose");
    nh_.param<std::string>(ros::this_node::getName()+"/fix_topic", fix_topic_, "/fix_topic");
    nh_.param<std::string>(ros::this_node::getName()+"/map_frame", map_frame_, "/map");
    joy_sub_ = nh_.subscribe(joy_topic_, 1 ,&waypoint_logger::joy_callback_, this);
    pose_sub_ = boost::make_shared<message_filters::Subscriber<geometry_msgs::PoseStamped> >(nh_, pose_topic_, 1);
    fix_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::NavSatFix> >(nh_, fix_topic_, 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<sync_polycy> >(sync_polycy(10), *pose_sub_, *fix_sub_);
    sync_->registerCallback(boost::bind(&waypoint_logger::pose_fix_callback_, this ,_1, _2));
}

waypoint_logger::~waypoint_logger()
{

}

void waypoint_logger::joy_callback_(const sensor_msgs::Joy::ConstPtr msg)
{
    mutex_.lock();
    mutex_.unlock();
    return;
}

void waypoint_logger::pose_fix_callback_(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::NavSatFixConstPtr& fix_msg)
{
    mutex_.lock();
    last_waypoints_->pose = *pose_msg;
    last_waypoints_->fix = *fix_msg;
    mutex_.unlock();
    return;
}