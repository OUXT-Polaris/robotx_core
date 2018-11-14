#include <world_pose_publisher.h>

world_pose_publisher::world_pose_publisher(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
}

world_pose_publisher::~world_pose_publisher()
{

}

void world_pose_publisher::gnss_callback_(const sensor_msgs::NavSatFixConstPtr& fix, 
    const geometry_msgs::TwistStampedConstPtr& twist,const geometry_msgs::QuaternionStampedConstPtr quat)
{
    
}