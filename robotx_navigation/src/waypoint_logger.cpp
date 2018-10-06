#include <waypoint_logger.h>

waypoint_logger::waypoint_logger() : tf_listener_(tf_buffer_)
{
    nh_.param<std::string>(ros::this_node::getName()+"/joy_topic", joy_topic_, "/joy");
    nh_.param<std::string>(ros::this_node::getName()+"/pose_topic", pose_topic_, "/robot_pose");
    nh_.param<std::string>(ros::this_node::getName()+"/fix_topic", fix_topic_, "/fix_topic");
    nh_.param<std::string>(ros::this_node::getName()+"/map_frame", map_frame_, "/map");
    joy_sub_ = nh_.subscribe(joy_topic_, 1 ,&waypoint_logger::joy_callback_, this);
    pose_sub_ = boost::make_shared<message_filters::Subscriber<geometry_msgs::PoseStamped> >(nh_, pose_topic_, 1);
    fix_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::NavSatFix> >(nh_, fix_topic_, 1);
    //message_filters::Synchronizer<sync_polycy> sync(sync_polycy(10), *pose_sub_, *fix_sub_);
    //sync.registerCallback(boost::bind(&waypoint_logger::pose_fix_callback_, this ,_1, _2));
    //sync(new message_filters::Synchronizer<sync_polycy>(sync_polycy(10), *pose_sub_, *fix_sub_));
    sync_ = boost::make_shared<message_filters::Synchronizer<sync_polycy> >(sync_polycy(10), *pose_sub_, *fix_sub_);
    //sync_ = boost::make_shared<message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, sensor_msgs::NavSatFix> >(*pose_sub_, *fix_sub_, 10);
    sync_->registerCallback(boost::bind(&waypoint_logger::pose_fix_callback_, this ,_1, _2));
}

waypoint_logger::~waypoint_logger()
{

}

void waypoint_logger::joy_callback_(const sensor_msgs::Joy::ConstPtr msg)
{
    mutex_.lock();
    last_logged_pose_ = last_pose_;
    if(last_logged_pose_.header.frame_id == map_frame_)
    {
        //waypoints_.push_back(last_logged_pose_);
    }
    mutex_.unlock();
    return;
}

void waypoint_logger::pose_fix_callback_(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::NavSatFixConstPtr& fix_msg)
{
    return;
}

/*
void waypoint_logger::pose_callback_(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    mutex_.lock();
    last_pose_ = *msg;
    mutex_.unlock();
    return;
}
*/