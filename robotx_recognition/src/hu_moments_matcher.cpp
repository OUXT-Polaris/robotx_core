#include <hu_moments_matcher.h>

hu_moments_matcher::hu_moments_matcher(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("robot_frame", image_topic_, ros::this_node::getName()+"/image_raw");
    
    cv::Mat circle_ref_image;
    cv::cvtColor(circle_ref_image, circle_ref_image, CV_RGB2GRAY);
    cv::adaptiveThreshold(circle_ref_image, circle_ref_image, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 0);
    std::vector<std::vector<cv::Point> > circle_contours;
    cv::findContours(circle_ref_image,circle_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    cv::Moments circle_ref_moments = cv::moments(circle_contours[0]);
    cv::HuMoments(circle_ref_moments, hu_circle_);

    cv::Mat triangle_ref_image;
    cv::cvtColor(triangle_ref_image, triangle_ref_image, CV_RGB2GRAY);
    cv::adaptiveThreshold(triangle_ref_image, triangle_ref_image, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 0);
    std::vector<std::vector<cv::Point> > triangle_contours;
    cv::findContours(triangle_ref_image,triangle_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    cv::Moments triangle_ref_moments = cv::moments(triangle_contours[0]);
    cv::HuMoments(triangle_ref_moments, hu_triangle_);

    cv::Mat rectangle_ref_image;
    cv::cvtColor(rectangle_ref_image, rectangle_ref_image, CV_RGB2GRAY);
    cv::adaptiveThreshold(rectangle_ref_image, rectangle_ref_image, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 0);
    std::vector<std::vector<cv::Point> > rectangle_contours;
    cv::findContours(rectangle_ref_image,rectangle_contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    cv::Moments rectangle_ref_moments = cv::moments(rectangle_contours[0]);
    cv::HuMoments(rectangle_ref_moments, hu_rectangle_);
}

hu_moments_matcher::~hu_moments_matcher()
{

}

boost::optional<std::pair<double,std::vector<cv::Point> > > hu_moments_matcher::match_(cv::Mat input, double hu_ref[7])
{
    std::pair<double,std::vector<cv::Point> > ret;
    cv::cvtColor(input, input, CV_RGB2GRAY);
    cv::adaptiveThreshold(input, input, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 0);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(input,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    if(contours.size() == 0)
    {
        return boost::none;
    }
    std::vector<double> match_rates(contours.size());
    for(int i = 0; i < contours.size(); i++)
    {
        cv::Moments moments = cv::moments(contours[i]);
        double hu_target[7];
        cv::HuMoments(moments, hu_target);
        if(matching_function_ == 0)
        {
            double ans;
            for(int m = 0; m<7; m++)
            {
                ans = ans + std::fabs(1/std::copysign(std::log(std::fabs(hu_target[m])),hu_ref[m])-1/std::copysign(std::log(std::fabs(hu_target[m])),hu_ref[m]));
            }
            match_rates[i] = ans;
        }
    }
    return ret;
}