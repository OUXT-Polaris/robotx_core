#include <hu_moments_matcher.h>

hu_moments_matcher::hu_moments_matcher(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("robot_frame", image_topic_, ros::this_node::getName()+"/image_raw");
}

hu_moments_matcher::~hu_moments_matcher()
{

}

double hu_moments_matcher::match_(cv::Mat input,cv::Mat ref)
{
    double ans;
    cv::cvtColor(input, input, CV_RGB2GRAY);
    cv::cvtColor(ref, ref, CV_RGB2GRAY);
    cv::adaptiveThreshold(input, input, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 0);
    cv::adaptiveThreshold(ref, ref, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 0);
    cv::findContours(input, input, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    std::vector<std::vector<cv::Point> > contours;
    for(int i = 0; i < contours.size(); i++)
    {
        cv::Moments moments = cv::moments(contours[i]);
        const int huMomentNum = 7;
        double lhuMoments[huMomentNum];
        cv::HuMoments(moments, lhuMoments);
    }
    double hua[7];
    double hub[7];
    if(matching_function_ == 0)
    {
        ans = 0;
        for(int i = 0; i<7; i++)
        {
            std::fabs(1/std::copysign(std::log(std::fabs(hua[i])),hua[i])-1/std::copysign(std::log(std::fabs(hub[i])),hub[i]));
        }
    }
    return ans;
}