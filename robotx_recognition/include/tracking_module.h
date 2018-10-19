#ifndef TRACKING_MODULE_H_INCLUDED
#define TRACKING_MODULE_H_INCLUDED

//headers in opencv
#include <opencv2/video/tracking.hpp>

//headers in STL
#include <memory>

class tracking_module
{
public:
    tracking_module();
    ~tracking_module();
private:
    std::shared_ptr<cv::KalmanFilter> kf_ptr_;
    cv::Mat state_;//x,y
    cv::Mat process_noise_;
    cv::Mat measurement_;
    cv::Mat transition_matrix_;
};

#endif  //TRACKING_MODULE_H_INCLUDED