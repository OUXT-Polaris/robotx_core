#include <obstacle_map_server.h>
#include <tf/transform_datatypes.h>

obstacle_map_server::obstacle_map_server() : params_(), tf_listener_(tf_buffer_) {
  obstacle_bbox_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/obstacle_map/bbox", 1);
  obstacle_map_pub_ = nh_.advertise<robotx_msgs::ObstacleMap>("/obstacle_map", 1);
  measurements_ = boost::circular_buffer<jsk_recognition_msgs::BoundingBoxArray>(params_.buffer_length);
  objects_bbox_sub_ =
      nh_.subscribe(params_.object_bbox_topic, 1, &obstacle_map_server::objects_bbox_callback_, this);
}

obstacle_map_server::~obstacle_map_server() {}

void obstacle_map_server::objects_bbox_callback_(jsk_recognition_msgs::BoundingBoxArray msg) {
  jsk_recognition_msgs::BoundingBoxArray transformed_bbox;
  geometry_msgs::TransformStamped transform_stemped;
  try
  {
    transform_stemped = tf_buffer_.lookupTransform(params_.map_frame, msg.header.frame_id, ros::Time(0));
  } catch (tf2::TransformException &ex)
  {
    ROS_WARN("Could NOT transform  %s", ex.what());
    return;
  }
  for(auto bbox_itr = msg.boxes.begin(); bbox_itr != msg.boxes.end(); bbox_itr++)
  {
    geometry_msgs::Vector3Stamped vector,vector_transformed;
    vector.header.frame_id = params_.map_frame;
    vector.header.stamp = bbox_itr->header.stamp;
    vector.vector = bbox_itr->dimensions;
    vector.vector.x = vector.vector.x + params_.margin * 2;
    vector.vector.y = vector.vector.y + params_.margin * 2;
    tf2::doTransform(vector,vector_transformed,transform_stemped);
    geometry_msgs::PoseStamped pose,pose_transformed;
    pose.header.frame_id = params_.map_frame;
    pose.header.stamp = bbox_itr->header.stamp;
    pose.pose = bbox_itr->pose;
    tf2::doTransform(pose,pose_transformed,transform_stemped);
    jsk_recognition_msgs::BoundingBox transformed_single_bbox;
    transformed_single_bbox.header.frame_id = params_.map_frame;
    transformed_single_bbox.header.stamp = bbox_itr->header.stamp;
    transformed_single_bbox.pose = pose_transformed.pose;
    transformed_single_bbox.dimensions = vector_transformed.vector;
    transformed_bbox.boxes.push_back(transformed_single_bbox);
  }
  measurements_.push_back(transformed_bbox);
  generate_obstacle_map_();
}

void obstacle_map_server::generate_obstacle_map_()
{
  robotx_msgs::ObstacleMap map;
  jsk_recognition_msgs::BoundingBoxArray bbox_msg;
  bbox_msg.header.frame_id = params_.map_frame;
  bbox_msg.header.stamp = ros::Time::now();
  for(auto measurements_itr = measurements_.begin(); measurements_itr != measurements_.end(); measurements_itr++)
  {
    for(auto bbox_itr = measurements_itr->boxes.begin(); bbox_itr != measurements_itr->boxes.end(); bbox_itr++)
    {
      bbox_msg.boxes.push_back(*bbox_itr);
    }
  }
  obstacle_bbox_pub_.publish(bbox_msg);
  return;
}