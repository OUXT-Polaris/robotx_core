#ifndef CNN_DETECTION_H
#define CNN_DETECTION_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

// messages
#include <robotx_msgs/ObjectRegionOfInterestArray.h>
#include <robotx_msgs/ObjectRegionOfInterest.h>
#include <sensor_msgs/Image.h>

class cnn_predictor {
  public:
    cnn_predictor();
    ~cnn_predictor();
    // callback
    void callback(const sensor_msgs::ImageConstPtr& image_msg,
                  const robotx_msgs::ObjectRegionOfInterestArrayConstPtr& rois_msg);

    // rosparam
    struct parameters {
      std::string roi_topic;
      std::string image_topic;
      std::string model_filename;
      std::string model_inputName;
      std::string model_outputName;
      int model_outputNum;
      bool use_mapped_memory;

      parameters() {
        /* model_outputNum = 0; */
        ros::param::param<std::string>(ros::this_node::getName() + "/roi_topic", roi_topic, "publisher/hogehoge");
        ros::param::param<std::string>(ros::this_node::getName() + "/image_topic", image_topic, "publisher/image");
        ros::param::param<std::string>(ros::this_node::getName() + "/model_filename", model_filename, "/home/ubuntu/tensorrt/resnet_test/resnet_v1_50_finetuned_4class_altered_model.plan");
        ros::param::param<std::string>(ros::this_node::getName() + "/model_inputName", model_inputName, "images");
        ros::param::param<std::string>(ros::this_node::getName() + "/model_outputName", model_outputName, "resnet_v1_50/SpatialSqueeze");
        ros::param::param<int>(ros::this_node::getName() + "/model_outputNum", model_outputNum, 4);
        ros::param::param<bool>(ros::this_node::getName() + "/use_mapped_memory", use_mapped_memory, false);
      }
    };
    // rosparam
    const struct parameters _params;

  private:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;

    // publisher
    ros::Publisher  _roi_pub;

    // subscriber and synchronizer
    message_filters::Subscriber<robotx_msgs::ObjectRegionOfInterestArray> _roi_sub;
    image_transport::SubscriberFilter _image_sub;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      robotx_msgs::ObjectRegionOfInterestArray> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> _sync;

    // functions
    robotx_msgs::ObjectRegionOfInterestArray _image_recognition(
        const robotx_msgs::ObjectRegionOfInterestArray rois,
        const cv::Mat image);
};
#endif
