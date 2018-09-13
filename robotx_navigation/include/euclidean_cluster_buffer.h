#ifndef EUCLIDEAN_CLUSTER_BUFFER_H_INCLUDED
#define EUCLIDEAN_CLUSTER_BUFFER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

//headers in STL
#include <vector>
#include <mutex>

//headers in boos
#include <boost/shared_ptr.hpp>

struct cluster_data
{
    const geometry_msgs::PointStamped point;
    const double radius;
    cluster_data(geometry_msgs::PointStamped p, double r) : point(p), radius(r){}
};

class euclidean_cluster_buffer
{
public:
    euclidean_cluster_buffer(double buffer_length);
    ~euclidean_cluster_buffer();
    std::vector<cluster_data> get_cluster_data();
    void add_cluster_data(cluster_data data);
private:
    std::mutex _mtx;
    std::vector<boost::shared_ptr<cluster_data> > _buffer;
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    ros::Duration _buffer_length;
};
#endif  //EUCLIDEAN_CLUSTER_BUFFER_H_INCLUDED