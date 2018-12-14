#ifndef POINTCLOUD_MERGER_H_INCLUDED
#define POINTCLOUD_MERGER_H_INCLUDED

#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// tf
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// headers in PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

// messages
#include <sensor_msgs/PointCloud2.h>

/**
 * @brief pointcloud_merger class
 *
 * merge 2 pointclouds and output as one pointcloud;
 *
 */
class pointcloud_merger {
  public:
    /**
     * @brief parameters for pointcloud_merger class
     */
    struct parameters {
      std::string pointcloud1_topic;
      std::string pointcloud2_topic;
      double voxelgrid_x,voxelgrid_y,voxelgrid_z;
      parameters() {
        ros::param::param<double>(ros::this_node::getName() + "/voxelgrid/x", voxelgrid_x, 0.1);
        ros::param::param<double>(ros::this_node::getName() + "/voxelgrid/y", voxelgrid_y, 0.1);
        ros::param::param<double>(ros::this_node::getName() + "/voxelgrid/z", voxelgrid_z, 0.1);
        ros::param::param<std::string>(ros::this_node::getName() + "/pointcloud1_topic", pointcloud1_topic, ros::this_node::getName() + "/pointcloud1");
        ros::param::param<std::string>(ros::this_node::getName() + "/pointcloud2_topic", pointcloud2_topic, ros::this_node::getName() + "/pointcloud2");
      }
    };
    const struct parameters _params;

    /**
     * @brief constructor
     */
    pointcloud_merger();
    ~pointcloud_merger();

    /**
     * @brief callback function for message_filter
     */
    void callback(
        const sensor_msgs::PointCloud2ConstPtr& pc1_msg,
        const sensor_msgs::PointCloud2ConstPtr& pc2_msg);

  private:
    /**
     * @brief tf buffer fir tf_listener_
     * @sa pointcloud_merger::tf_listener_
     */
    tf2_ros::Buffer tf_buffer_;
      /**
       * @brief tf listener for
       * output_frame -> pointcloud1_topic frame
       * and
       * output_frame -> pointcloud2_topic frame
       */
    tf2_ros::TransformListener tf_listener_;

    /**
     * @brief ROS Nodehandle
     */
    ros::NodeHandle _nh;

    /**
     * @brief ROS publisher for ~/merged_points (message type : sensor_msgs/PointCloud2)
     */
    ros::Publisher  _pc_pub;

    /**
     * @brief message_filter subscriber and sync
     */
    message_filters::Subscriber<sensor_msgs::PointCloud2> _pc1_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> _pc2_sub;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> _sync;
};

#endif  // POINTCLOUD_MERGER_H_INCLUDED
