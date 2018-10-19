#include <tracking_module.h>

tracking_module::tracking_module()
{
    kf_ptr_ = std::make_shared<cv::KalmanFilter>(2,2,0,CV_32F);
    state_ = cv::Mat(2, 1, CV_32F);
    process_noise_ = cv::Mat(2, 1, CV_32F);
    measurement_ = cv::Mat(2, 1, CV_32F);
    transition_matrix_ = (cv::Mat_<float>(2, 2) << 1, 0, 0, 1);
    cv::setIdentity(kf_ptr_->measurementMatrix);
    cv::setIdentity(kf_ptr_->processNoiseCov, cv::Scalar::all(1e-5));
    cv::setIdentity(kf_ptr_->measurementNoiseCov, cv::Scalar::all(1e-1));
    cv::setIdentity(kf_ptr_->errorCovPost, cv::Scalar::all(1));
}

tracking_module::~tracking_module()
{

}