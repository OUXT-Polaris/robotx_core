#ifndef HUE_MOMENTS_METCHER_H_INCLUDED
#define HUE_MOMENTS_METCHER_H_INCLUDED

#include <ros/ros.h>
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

// headers in opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//headers in boost
#include <boost/optional.hpp>

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
    double hu_circle_[7];
    double hu_rectangle_[7];
    double hu_triangle_[7];
    boost::optional<std::pair<double,std::vector<cv::Point> > > match_(cv::Mat input, double hu_ref[7]);
};

#endif  //HUE_MOMENTS_METCHER_H_INCLUDED