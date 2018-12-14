// headers in this package
#include <pointcloud_merger.h>

// headers in STL
#include <mutex>

pointcloud_merger::pointcloud_merger() :
  _params(),
  _pc1_sub(_nh, _params.pointcloud1_topic, 1),
  _pc2_sub(_nh, _params.pointcloud2_topic, 1),
  _sync(SyncPolicy(10), _pc1_sub, _pc2_sub),
  tf_listener_(tf_buffer_)
{
  // publisher
  _pc_pub = _nh.advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/merged_points", 1);

  // message_filter
  _sync.registerCallback(boost::bind(&pointcloud_merger::callback, this, _1, _2));
}

// destructor
pointcloud_merger::~pointcloud_merger() {
}

void pointcloud_merger::callback(
    const sensor_msgs::PointCloud2ConstPtr& pc1_msg,
    const sensor_msgs::PointCloud2ConstPtr& pc2_msg) {
  sensor_msgs::PointCloud2 pc1 = *pc1_msg;
  sensor_msgs::PointCloud2 pc2 = *pc2_msg;
  if (pc1_msg->header.frame_id == "" || pc2_msg->header.frame_id == "") {
    return;
  }
  if (pc1_msg->header.frame_id != pc2_msg->header.frame_id) {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped = tf_buffer_.lookupTransform(
        pc1_msg->header.frame_id,
        pc2_msg->header.frame_id,
        ros::Time(0));
    tf2::doTransform(pc2, pc2, transform_stamped);
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud1(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*pc1_msg, *pcl_cloud1);
  pcl::fromROSMsg(*pc2_msg, *pcl_cloud2);
  *pcl_output_cloud = *pcl_cloud1 + *pcl_cloud2;
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(pcl_output_cloud);
  sor.setLeafSize(_params.voxelgrid_x,_params.voxelgrid_y,_params.voxelgrid_z);
  sor.filter(*pcl_output_cloud);
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*pcl_output_cloud, output_msg);
  _pc_pub.publish(output_msg);
  return;
}

