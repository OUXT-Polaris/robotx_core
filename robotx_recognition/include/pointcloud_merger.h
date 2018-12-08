#ifndef POINTCLOUD_MERGER_H
#define POINTCLOUD_MERGER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// headers in PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// messages
#include <robotx_msgs/ObjectRegionOfInterestArray.h>
#include <robotx_msgs/ObjectRegionOfInterest.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

class pointcloud_merger {
  public:
    pointcloud_merger();
    ~pointcloud_merger();
    // callback
    void callback(
        const sensor_msgs::PointCloud2ConstPtr& pc1_msg,
        const sensor_msgs::PointCloud2ConstPtr& pc2_msg);
    // rosparam
    struct parameters {
      std::string pointcloud1_topic;
      std::string pointcloud2_topic;
      parameters() {
        ros::param::param<std::string>(ros::this_node::getName() + "/pointcloud1_topic", pointcloud1_topic, ros::this_node::getName() + "/pointcloud1");
        ros::param::param<std::string>(ros::this_node::getName() + "/pointcloud2_topic", pointcloud2_topic, ros::this_node::getName() + "/pointcloud2");
      }
    };
    // rosparam
    const struct parameters _params;

  private:
    ros::NodeHandle _nh;

    // publisher
    ros::Publisher  _pc_pub;

    // subscriber and synchronizer
    message_filters::Subscriber<sensor_msgs::PointCloud2> _pc1_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> _pc2_sub;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> _sync;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
#endif
