#include <world_pose_publisher.h>

world_pose_publisher::world_pose_publisher(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("fix_topic", fix_topic_, ros::this_node::getName()+"/fix");
    pnh_.param<std::string>("twist_topic", twist_topic_, ros::this_node::getName()+"/twist");
    pnh_.param<std::string>("true_course_topic", true_course_topic_, ros::this_node::getName()+"/true_course");
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("world_pose_topic", world_pose_topic_, ros::this_node::getName()+"/world_pose");
    pnh_.param<std::string>("world_odom_topic", world_odom_topic_, ros::this_node::getName()+"/odom");
    world_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(world_odom_topic_,10);
    world_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(world_pose_topic_,10);
    origin_ = boost::none;
    origin_utm_ = boost::none;
    fix_sub_ptr_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::NavSatFix> >(nh_,fix_topic_,1);
    twist_sub_ptr_ = boost::make_shared<message_filters::Subscriber<geometry_msgs::TwistStamped> >(nh_,twist_topic_,1);
    true_course_sub_ptr_ = boost::make_shared<message_filters::Subscriber<geometry_msgs::QuaternionStamped> >(nh_,true_course_topic_,1);
    sync_ptr_ = boost::make_shared<message_filters::Synchronizer<sync_policy> >(sync_policy(10),*fix_sub_ptr_,*twist_sub_ptr_,*true_course_sub_ptr_);
    sync_ptr_->registerCallback(boost::bind(&world_pose_publisher::gnss_callback_,this,_1,_2,_3));
}

world_pose_publisher::~world_pose_publisher()
{

}

void world_pose_publisher::gnss_callback_(const sensor_msgs::NavSatFixConstPtr& fix, 
    const geometry_msgs::TwistStampedConstPtr& twist,const geometry_msgs::QuaternionStampedConstPtr true_course)
{
    geometry_msgs::PoseStamped world_pose;
    nav_msgs::Odometry world_odom;
    world_pose.header.stamp = fix->header.stamp;
    world_odom.header.stamp = fix->header.stamp;
    world_pose.header.frame_id = world_frame_;
    world_odom.header.frame_id = world_frame_;
    world_odom.child_frame_id = twist->header.frame_id;
    world_odom.twist.twist = twist->twist;
    if(!origin_)
    {
        global_pose init_pose;
        init_pose.fix = *fix;
        init_pose.true_course = *true_course;
        origin_ = init_pose;
        double x,y;
        LatLonToUTMXY(init_pose.fix.latitude,init_pose.fix.altitude,0,x,y);
    }
    else
    {
    }
    world_odom.pose.pose = world_pose.pose;
    world_pose_pub_.publish(world_pose);
    world_odom_pub_.publish(world_odom);
}