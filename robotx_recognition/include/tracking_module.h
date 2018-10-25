#ifndef TRACKING_MODULE_H_INCLUDED
#define TRACKING_MODULE_H_INCLUDED

//headers in opencv
#include <opencv2/video/tracking.hpp>

//headers in STL
#include <memory>

//headers in boost
#include <boost/optional.hpp>

//headers in ROS
#include<jsk_recognition_msgs/BoundingBox.h>

class tracking_module
{
public:
    tracking_module(std::string map_frame,int id);
    ~tracking_module();
    jsk_recognition_msgs::BoundingBox input_measurement(boost::optional<jsk_recognition_msgs::BoundingBox> measurement, ros::Time stamp);
    jsk_recognition_msgs::BoundingBox get_predicted_bbox();
    const int target_id;
private:
    std::shared_ptr<cv::KalmanFilter> kf_ptr_;
    cv::Mat state_;//x,y
    cv::Mat process_noise_;
    std::string map_frame_;
    jsk_recognition_msgs::BoundingBox latest_measurement_;
    jsk_recognition_msgs::BoundingBox predicted_bbox_;
};

#endif  //TRACKING_MODULE_H_INCLUDED