#include <obstacle_map_server.h>
#include <tf/transform_datatypes.h>

obstacle_map_server::obstacle_map_server() : params_(), tf_listener_(tf_buffer_) {
  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/obstacle_map/marker", 1);
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
  visualization_msgs::MarkerArray marker_array_;
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer_.lookupTransform(params_.robot_frame, params_.map_frame, ros::Time(0), ros::Duration(1));
  }
  catch(tf2::TransformException &ex)
  {
    ROS_WARN("%s",ex.what());
    return;
  }
  map.header.frame_id = params_.robot_frame;
  ros::Time now = ros::Time::now();
  map.header.stamp = now;
  int id = 0;
  for(auto measurements_itr = measurements_.begin(); measurements_itr != measurements_.end(); measurements_itr++)
  {
    for(auto bbox_itr = measurements_itr->boxes.begin(); bbox_itr != measurements_itr->boxes.end(); bbox_itr++)
    {
      geometry_msgs::PoseStamped transformed_pose;
      geometry_msgs::PoseStamped obstacle_pose;
      obstacle_pose.header = measurements_itr->header;
      obstacle_pose.pose = bbox_itr->pose;
      tf2::doTransform(obstacle_pose, transformed_pose, transform_stamped);
      map.points.push_back(transformed_pose.pose.position);
      visualization_msgs::Marker marker;
      marker.header.stamp = now;
      marker.header.frame_id = params_.robot_frame;
      marker.type = marker.CYLINDER;
      marker.action = marker.ADD;
      marker.id = id;
      marker.pose.position.x = bbox_itr->pose.position.x;
      marker.pose.position.y = bbox_itr->pose.position.y;
      marker.pose.orientation.w = 1;
      if(bbox_itr->dimensions.x > bbox_itr->dimensions.y)
      {
        map.radius.push_back(bbox_itr->dimensions.x);
        marker.scale.x = bbox_itr->dimensions.x;
        marker.scale.y = bbox_itr->dimensions.x;
      }
      else
      {
        map.radius.push_back(bbox_itr->dimensions.y);
        marker.scale.x = bbox_itr->dimensions.y;
        marker.scale.x = bbox_itr->dimensions.y;
      }
      marker.color.r = 1;
      marker.color.g = 1;
      marker.color.b = 0;
      marker.color.a = 1;
      marker_array_.markers.push_back(marker);
      id++;
    }
  }
  obstacle_map_pub_.publish(map);
  marker_pub_.publish(marker_array_);
  return;
}