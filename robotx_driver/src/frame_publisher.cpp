#include <frame_publisher.h>

frame_publisher::frame_publisher(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    callback_func_type_ = boost::bind(&frame_publisher::configure_callback_, this, _1, _2);
    server_.setCallback(callback_func_type_);
}

frame_publisher::~frame_publisher()
{

}

void frame_publisher::configure_callback_(robotx_driver::frame_publisherConfig &config, uint32_t level)
{
    ros::Time now = ros::Time::now();

    geometry_msgs::TransformStamped transform_stamped_ai_pilot_;
    transform_stamped_ai_pilot_.header.frame_id = "base_link";
    transform_stamped_ai_pilot_.header.stamp = now;
    transform_stamped_ai_pilot_.child_frame_id = "ai_pilot";
    transform_stamped_ai_pilot_.transform.translation.x = config.ai_pilot_x;
    transform_stamped_ai_pilot_.transform.translation.y = config.ai_pilot_y;
    transform_stamped_ai_pilot_.transform.translation.z = config.ai_pilot_z;
    transform_stamped_ai_pilot_.transform.rotation = convert(config.ai_pilot_roll*M_PI,config.ai_pilot_pitch*M_PI,config.ai_pilot_yaw*M_PI);
    broadcaster_.sendTransform(transform_stamped_ai_pilot_);

    geometry_msgs::TransformStamped transform_stamped_front_velodyne_;
    transform_stamped_front_velodyne_.header.frame_id = "base_link";
    transform_stamped_front_velodyne_.header.stamp = now;
    transform_stamped_front_velodyne_.child_frame_id = "front_velodyne";
    transform_stamped_front_velodyne_.transform.translation.x = config.front_velodyne_x;
    transform_stamped_front_velodyne_.transform.translation.y = config.front_velodyne_y;
    transform_stamped_front_velodyne_.transform.translation.z = config.front_velodyne_z;
    transform_stamped_front_velodyne_.transform.rotation = convert(config.front_velodyne_roll*M_PI,config.front_velodyne_pitch*M_PI,config.front_velodyne_yaw*M_PI);
    broadcaster_.sendTransform(transform_stamped_front_velodyne_);

    geometry_msgs::TransformStamped transform_stamped_front_camera_;
    transform_stamped_front_camera_.header.frame_id = "base_link";
    transform_stamped_front_camera_.header.stamp = now;
    transform_stamped_front_camera_.child_frame_id = "front_camera_link";
    transform_stamped_front_camera_.transform.translation.x = config.front_camera_x;
    transform_stamped_front_camera_.transform.translation.y = config.front_camera_y;
    transform_stamped_front_camera_.transform.translation.z = config.front_camera_z;
    transform_stamped_front_camera_.transform.rotation = convert(config.front_camera_roll*M_PI,config.front_camera_pitch*M_PI,config.front_camera_yaw*M_PI);
    broadcaster_.sendTransform(transform_stamped_front_camera_);

    geometry_msgs::TransformStamped transform_stamped_gps_;
    transform_stamped_gps_.header.frame_id = "base_link";
    transform_stamped_gps_.header.stamp = now;
    transform_stamped_gps_.child_frame_id = "gps";
    transform_stamped_gps_.transform.translation.x = config.gps_x;
    transform_stamped_gps_.transform.translation.y = config.gps_y;
    transform_stamped_gps_.transform.translation.z = config.gps_z;
    transform_stamped_gps_.transform.rotation = convert(config.gps_roll*M_PI,config.gps_pitch*M_PI,config.gps_yaw*M_PI);
    broadcaster_.sendTransform(transform_stamped_gps_);

    geometry_msgs::TransformStamped transform_stamped_imu_;
    transform_stamped_imu_.header.frame_id = "base_link";
    transform_stamped_imu_.header.stamp = now;
    transform_stamped_imu_.child_frame_id = "imu";
    transform_stamped_imu_.transform.translation.x = config.imu_x;
    transform_stamped_imu_.transform.translation.y = config.imu_y;
    transform_stamped_imu_.transform.translation.z = config.imu_z;
    transform_stamped_imu_.transform.rotation = convert(config.imu_roll*M_PI,config.imu_pitch*M_PI,config.imu_yaw*M_PI);
    broadcaster_.sendTransform(transform_stamped_imu_);
}

geometry_msgs::Quaternion frame_publisher::convert(double r,double p, double y)
{
    geometry_msgs::Quaternion ret;
    tf::Quaternion quat = tf::createQuaternionFromRPY(r,p,y);
    quaternionTFToMsg(quat,ret);
    return ret;
}