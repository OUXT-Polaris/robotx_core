#ifndef HUE_MOMENTS_METCHER_H_INCLUDED
#define HUE_MOMENTS_METCHER_H_INCLUDED

//headers in this package
#include <robotx_msgs/ObjectRegionOfInterestArray.h>

//headers int ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// headers in opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//headers in boost
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

class hu_moments_matcher
{
    struct parameters
    {
        std::string image_topic;
        std::string roi_topic;
        parameters()
        {
            ros::param::param<std::string>(ros::this_node::getName() + "/roi_topic", roi_topic, ros::this_node::getName() + "/roi");
            ros::param::param<std::string>(ros::this_node::getName() + "/image_topic", image_topic, ros::this_node::getName() + "/image");
        }
    };

public:
    hu_moments_matcher(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~hu_moments_matcher();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    parameters params;
    //std::string image_topic_;
    int matching_function_;
    double hu_circle_[7];
    double hu_rectangle_[7];
    double hu_triangle_[7];
    std::pair<double,std::vector<cv::Point> > match_(cv::Mat input, double hu_ref[7]);

    // subscriber and synchronizer
    message_filters::Subscriber<robotx_msgs::ObjectRegionOfInterestArray> roi_sub_;
    image_transport::SubscriberFilter image_sub_;
    image_transport::ImageTransport it_;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      robotx_msgs::ObjectRegionOfInterestArray> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync_;
    void callback_(const sensor_msgs::ImageConstPtr& image_msg,const robotx_msgs::ObjectRegionOfInterestArrayConstPtr& rois_msg);
    std::vector<cv::Rect> get_rois_(const robotx_msgs::ObjectRegionOfInterestArrayConstPtr& rois_msg);
};

#endif  //HUE_MOMENTS_METCHER_H_INCLUDED