#include <waypoint_logger.h>

waypoint_logger::waypoint_logger() : tf_listener_(tf_buffer_)
{
    waypoint_recieved_ = false;
    button_pressed_ = false;
    waypoint_id_ = 0;
    nh_.param<std::string>(ros::this_node::getName()+"/joy_topic", joy_topic_, "/joy");
    nh_.param<std::string>(ros::this_node::getName()+"/pose_topic", pose_topic_, "/robot_pose");
    nh_.param<std::string>(ros::this_node::getName()+"/fix_topic", fix_topic_, "/fix_topic");
    nh_.param<std::string>(ros::this_node::getName()+"/map_frame", map_frame_, "/map");
    nh_.param<int>(ros::this_node::getName()+"/joy_button_index", joy_button_index_, 2);
    joy_sub_ = nh_.subscribe(joy_topic_, 1 ,&waypoint_logger::joy_callback_, this);
    pose_sub_ = boost::make_shared<message_filters::Subscriber<geometry_msgs::PoseStamped> >(nh_, pose_topic_, 1);
    fix_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::NavSatFix> >(nh_, fix_topic_, 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<sync_polycy> >(sync_polycy(10), *pose_sub_, *fix_sub_);
    sync_->registerCallback(boost::bind(&waypoint_logger::pose_fix_callback_, this ,_1, _2));
}

waypoint_logger::~waypoint_logger()
{
    std::string path = ros::package::getPath("robotx_navigation") + "/data/waypoints.bag";
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Write);
    bag.write("waypoints", ros::Time::now(), waypoints_);
    bag.close();
}

void waypoint_logger::joy_callback_(const sensor_msgs::Joy::ConstPtr msg)
{
    mutex_.lock();
    if(msg->buttons[joy_button_index_] == 1 && button_pressed_ == false)
    {
        button_pressed_ = true;
        if(waypoint_recieved_ == true)
        {
            if(last_waypoint_.pose.header.frame_id == map_frame_)
            {
                last_waypoint_.id = waypoint_id_;
                waypoints_.waypoints.push_back(last_waypoint_);
                waypoint_id_ = waypoint_id_ + 1;
            }
            else
            {
                geometry_msgs::TransformStamped transform_stamped_;
                try
                {
                    last_waypoint_.id = waypoint_id_;
                    transform_stamped_ = tf_buffer_.lookupTransform(map_frame_, last_waypoint_.pose.header.frame_id, ros::Time(0));
                    tf2::doTransform(last_waypoint_.pose, last_waypoint_.pose, transform_stamped_);
                    waypoints_.waypoints.push_back(last_waypoint_);
                    waypoint_id_ = waypoint_id_ + 1;
                }
                catch (tf2::TransformException &ex)
                {
                    ROS_WARN("%s",ex.what());
                    return;
                }
            }
        }
    }
    else if(msg->buttons[joy_button_index_] == 0)
    {
        button_pressed_ = false;
    }
    mutex_.unlock();
    return;
}

void waypoint_logger::pose_fix_callback_(const geometry_msgs::PoseStampedConstPtr& pose_msg, const sensor_msgs::NavSatFixConstPtr& fix_msg)
{
    mutex_.lock();
    last_waypoint_.pose = *pose_msg;
    last_waypoint_.fix = *fix_msg;
    waypoint_recieved_ = true;
    mutex_.unlock();
    return;
}