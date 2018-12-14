#include <gps_to_base_link.h>

gps_to_base_link::gps_to_base_link(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    callback_func_type_ = boost::bind(&gps_to_base_link::configure_callback_, this, _1, _2);
    server_.setCallback(callback_func_type_);
}

gps_to_base_link::~gps_to_base_link()
{

}

void gps_to_base_link::configure_callback_(robotx_driver::gps_to_base_linkConfig &config, uint32_t level)
{
    ros::Time now = ros::Time::now();
    
    geometry_msgs::TransformStamped transform_stamped_gps_;
    transform_stamped_gps_.header.frame_id = "gps";
    transform_stamped_gps_.header.stamp = now;
    transform_stamped_gps_.child_frame_id = "base_link";
    transform_stamped_gps_.transform.translation.x = config.gps_x;
    transform_stamped_gps_.transform.translation.y = config.gps_y;
    transform_stamped_gps_.transform.translation.z = config.gps_z;
    transform_stamped_gps_.transform.rotation = convert(config.gps_roll*M_PI,config.gps_pitch*M_PI,config.gps_yaw*M_PI);
    broadcaster_.sendTransform(transform_stamped_gps_);
}

geometry_msgs::Quaternion gps_to_base_link::convert(double r,double p, double y)
{
    geometry_msgs::Quaternion ret;
    tf::Quaternion quat = tf::createQuaternionFromRPY(r,p,y);
    quaternionTFToMsg(quat,ret);
    return ret;
}