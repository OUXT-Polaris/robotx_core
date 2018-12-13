#include <world_pose_publisher.h>

world_pose_publisher::world_pose_publisher(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    data_recieved_ = false;
    pnh_.param<std::string>("fix_topic", fix_topic_, ros::this_node::getName()+"/fix");
    pnh_.param<std::string>("gps_twist_topic", gps_twist_topic_, ros::this_node::getName()+"/twist");
    pnh_.param<std::string>("true_course_topic", true_course_topic_, ros::this_node::getName()+"/true_course");
    pnh_.param<std::string>("world_frame", world_frame_, "world");
    pnh_.param<std::string>("robot_frame", robot_frame_, "base_link");
    pnh_.param<std::string>("world_pose_topic", world_pose_topic_, ros::this_node::getName()+"/world_pose");
    pnh_.param<std::string>("world_odom_topic", world_odom_topic_, ros::this_node::getName()+"/odom");
    pnh_.param<std::string>("imu_topic", imu_topic_, ros::this_node::getName()+"/imu");
    pnh_.param<double>("publish_rate", publish_rate_, 10);
    world_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(world_odom_topic_,10);
    world_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(world_pose_topic_,10);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/vel",10);
    fix_sub_ptr_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::NavSatFix> >(nh_,fix_topic_,1);
    twist_sub_ptr_ = boost::make_shared<message_filters::Subscriber<geometry_msgs::TwistStamped> >(nh_,gps_twist_topic_,1);
    true_course_sub_ptr_ = boost::make_shared<message_filters::Subscriber<geometry_msgs::QuaternionStamped> >(nh_,true_course_topic_,1);
    sync_ptr_ = boost::make_shared<message_filters::Synchronizer<sync_policy> >(sync_policy(10),*fix_sub_ptr_,*twist_sub_ptr_,*true_course_sub_ptr_);
    sync_ptr_->registerCallback(boost::bind(&world_pose_publisher::gnss_callback_,this,_1,_2,_3));
    imu_sub_ = nh_.subscribe(imu_topic_,10,&world_pose_publisher::imu_callback_,this);
}

world_pose_publisher::~world_pose_publisher()
{

}

void world_pose_publisher::run()
{
    boost::thread publish_thread(&world_pose_publisher::publish_world_frame_,this);
    return;
}

void world_pose_publisher::publish_world_frame_()
{
    ros::Rate rate(publish_rate_);
    while(ros::ok())
    {
        mtx_.lock();
        if(!data_recieved_)
        {
            mtx_.unlock();
            rate.sleep();
            continue;
        }
        geometry_msgs::PoseStamped world_pose;
        nav_msgs::Odometry world_odom;
        world_pose.header.stamp = fix_.header.stamp;
        world_odom.header.stamp = fix_.header.stamp;
        world_pose.header.frame_id = world_frame_;
        world_odom.header.frame_id = world_frame_;
        world_odom.child_frame_id = twist_header_.frame_id;
        twist_.twist.angular.z = yawrate_;
        twist_.twist.linear.x = v_ + dv_;
        world_odom.twist.twist = twist_.twist;
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = fix_.header.stamp;
        transform_stamped.header.frame_id = "world";
        transform_stamped.child_frame_id = fix_.header.frame_id;
        geographic_msgs::GeoPose geo_pose = geodesy::toMsg(fix_,true_course_.quaternion);
        geodesy::UTMPose utm_pose;
        geodesy::fromMsg(geo_pose, utm_pose);
        geometry_msgs::Pose pose = geodesy::toGeometry(utm_pose);
        double roll,pitch,yaw;
        get_rpy_(pose.orientation,roll,pitch,yaw);
        yaw = yaw + theta_trans_imu_;
        get_quat_(roll,pitch,yaw,pose.orientation);

        transform_stamped.transform.translation.x = pose.position.x - std::sin(yaw + theta_trans_imu_/2)*x_trans_imu_ - std::cos(yaw + theta_trans_imu_/2)*y_trans_imu_;
        transform_stamped.transform.translation.y = pose.position.y + std::cos(yaw + theta_trans_imu_/2)*x_trans_imu_ - std::sin(yaw + theta_trans_imu_/2)*y_trans_imu_;
        transform_stamped.transform.translation.z = 0;
        transform_stamped.transform.rotation = pose.orientation;
        broadcaster_.sendTransform(transform_stamped);
        world_pose.pose.position.x = pose.position.x - std::sin(yaw + theta_trans_imu_/2)*x_trans_imu_ - std::cos(yaw + theta_trans_imu_/2)*y_trans_imu_;
        world_pose.pose.position.y = pose.position.y + std::cos(yaw + theta_trans_imu_/2)*x_trans_imu_ - std::sin(yaw + theta_trans_imu_/2)*y_trans_imu_;
        world_pose.pose.position.z = 0;
        world_pose.pose.orientation = pose.orientation;
        world_odom.pose.pose = world_pose.pose;
        world_pose_pub_.publish(world_pose);
        world_odom_pub_.publish(world_odom);
        twist_pub_.publish(world_odom.twist.twist);
        mtx_.unlock();
        rate.sleep();
    }
    return;
}

void world_pose_publisher::gnss_callback_(const sensor_msgs::NavSatFixConstPtr& fix, 
    const geometry_msgs::TwistStampedConstPtr& twist,const geometry_msgs::QuaternionStampedConstPtr true_course)
{
    mtx_.lock();
    data_recieved_ = true;
    fix_ = *fix;
    twist_ = *twist;
    twist_header_ = twist->header;
    true_course_ = *true_course;
    imu_reset_flag_ = true;
    x_trans_imu_ = 0;
    y_trans_imu_ = 0;
    theta_trans_imu_ = 0;
    dv_ = 0;
    v_ = twist->twist.linear.x;
    mtx_.unlock();
}

void world_pose_publisher::imu_callback_(const sensor_msgs::Imu::ConstPtr msg)
{
    mtx_.lock();
    if(!data_recieved_)
    {
        mtx_.unlock();
        return;
    }
    if(!last_imu_timestamp_)
    {
        last_imu_timestamp_ = msg->header.stamp;
        mtx_.unlock();
        return;
    }
    double dt = (msg->header.stamp - *last_imu_timestamp_).toSec();
    yawrate_ = msg->angular_velocity.z;
    theta_trans_imu_ = theta_trans_imu_ + msg->angular_velocity.z * dt;
    dv_ = dv_ + msg->linear_acceleration.x * dt;
    x_trans_imu_ = x_trans_imu_ - msg->linear_acceleration.x * std::cos(theta_trans_imu_) * dt * 0.1 + twist_.twist.linear.x * std::cos(theta_trans_imu_) * dt * 0.5;
    y_trans_imu_ = y_trans_imu_ - msg->linear_acceleration.y * std::sin(theta_trans_imu_) * dt * 0.1 + twist_.twist.linear.x * std::sin(theta_trans_imu_) * dt * 0.5;
    last_imu_timestamp_ = msg->header.stamp;
    mtx_.unlock();
    return;
}

void world_pose_publisher::get_rpy_(const geometry_msgs::Quaternion &q, double &roll,double &pitch,double &yaw)
{
    tf::Quaternion quat(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return;
}

void world_pose_publisher::get_quat_(double roll,double pitch,double yaw,geometry_msgs::Quaternion &q)
{
    tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
    quaternionTFToMsg(quat,q);
    return;
}