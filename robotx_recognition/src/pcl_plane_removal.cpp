#include <pcl_plane_removal.h>

pcl_plane_removal::pcl_plane_removal() : params_() {
  pcl_input_sub_ =
      nh_.subscribe(params_.input_pcl_topic, 1, &pcl_plane_removal::cloud_CB, this);
  modified_pcl_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(params_.output_pcl_topic, 1);
  ROS_INFO("DistanceThreshold = %f",params_.DistanceThreshold);
  ROS_INFO("MaxIterations = %d",params_.MaxIterations);
  ROS_INFO("Probability = %f",params_.Probability);
      
}

pcl_plane_removal::~pcl_plane_removal() {}

void pcl_plane_removal::cloud_CB(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg (*msg, *cloud);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setInputCloud(cloud);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    //param
    seg.setDistanceThreshold (params_.DistanceThreshold);
    seg.setMaxIterations(params_.MaxIterations);
    seg.setProbability(params_.Probability);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud);

    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(*cloud, pub_cloud);
    
    modified_pcl_pub_.publish (pub_cloud);

}