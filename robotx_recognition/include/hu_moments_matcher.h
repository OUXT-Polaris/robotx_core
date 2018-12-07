#ifndef HUE_MOMENTS_METCHER_H_INCLUDED
#define HUE_MOMENTS_METCHER_H_INCLUDED

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// headers in opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class hu_moments_matcher
{
public:
    hu_moments_matcher(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~hu_moments_matcher();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string image_topic_;
    int matching_function_;
    double match_(cv::Mat input,cv::Mat ref);
};

#endif  //HUE_MOMENTS_METCHER_H_INCLUDED