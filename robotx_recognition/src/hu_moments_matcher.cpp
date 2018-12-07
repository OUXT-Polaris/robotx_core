#include <hu_moments_matcher.h>

hu_moments_matcher::hu_moments_matcher(ros::NodeHandle nh,ros::NodeHandle pnh) :
  it_(nh),
  nh_(nh),
  pnh_(pnh),
  params(),
  image_sub_(it_, params.image_topic, 1),
  roi_sub_(nh_, params.roi_topic, 1),
  sync_(SyncPolicy(1), image_sub_, roi_sub_)
{
    sync_.registerCallback(boost::bind(&hu_moments_matcher::callback_, this, _1, _2));

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

void hu_moments_matcher::callback_(const sensor_msgs::ImageConstPtr& image_msg,const robotx_msgs::ObjectRegionOfInterestArrayConstPtr& rois_msg)
{
    cv::Mat image;
    try
    {
        image = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8)->image;
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB, 3);
    } catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    std::vector<cv::Rect> rects = get_rois_(rois_msg);
    for(auto rect_itr = rects.begin(); rect_itr != rects.end(); rect_itr++)
    {
        cv::Mat roi_image = image(*rect_itr);
        std::vector<double> values(3);
        std::pair<double,std::vector<cv::Point> > result_circle = match_(roi_image, hu_circle_);
        values[0] = result_circle.first;
        std::pair<double,std::vector<cv::Point> > result_triangle = match_(roi_image, hu_triangle_);
        values[1] = result_triangle.first;
        std::pair<double,std::vector<cv::Point> > result_rectangle = match_(roi_image, hu_rectangle_);
        values[2] = result_rectangle.first;
        int index = *std::max_element(values.begin(), values.end());
        if(values[index] > 0)
        {
            
        }
    }
    return;
}

std::vector<cv::Rect> hu_moments_matcher::get_rois_(const robotx_msgs::ObjectRegionOfInterestArrayConstPtr& rois_msg)
{
    std::vector<cv::Rect> rects;
    for(auto rect_itr = rois_msg->object_rois.begin(); rect_itr != rois_msg->object_rois.end(); rect_itr++)
    {
        cv::Rect rect(rect_itr->roi_2d.x_offset,rect_itr->roi_2d.y_offset,rect_itr->roi_2d.width,rect_itr->roi_2d.height);
        rects.push_back(rect);
    }
    return rects;
}

std::pair<double,std::vector<cv::Point> > hu_moments_matcher::match_(cv::Mat input, double hu_ref[7])
{
    std::pair<double,std::vector<cv::Point> > ret;
    cv::cvtColor(input, input, CV_RGB2GRAY);
    cv::adaptiveThreshold(input, input, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 7, 0);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(input,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    if(contours.size() == 0)
    {
        ret.first = -1;
        return ret;
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