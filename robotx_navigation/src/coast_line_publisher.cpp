#include <coast_line_publisher.h>

coast_line_publisher::coast_line_publisher(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<double>("range", range_, 0.0);
    pnh_.param<std::string>("fix_topic", fix_topic_, ros::this_node::getName()+"/fix");
    pnh_.param<std::string>("geographic_map_topic", geographic_map_topic_, ros::this_node::getName()+"/geographic_map");
    coast_line_pub_ = pnh_.advertise<robotx_msgs::CoastLineArray>("coast_lines",10);
    marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("marker",10);
    utm_area_buf_ = boost::circular_buffer<int>(2);
    fix_sub_ = nh_.subscribe(fix_topic_,10,&coast_line_publisher::fix_callback_,this);
    geographic_map_sub_ = nh_.subscribe(geographic_map_topic_,10,&coast_line_publisher::map_callback_,this);
}

coast_line_publisher::~coast_line_publisher()
{

}

boost::optional<point_with_area> coast_line_publisher::query_point_(int node_id, std::vector<point_with_area> points_in_area)
{
    point_with_area point;
    for(auto point_itr = points_in_area.begin(); point_itr != points_in_area.end(); point_itr++)
    {
        if(point_itr->node_id == node_id)
        {
            return point;
        }
    }
    return boost::none;
}

point_with_area coast_line_publisher::convert_geopoint_(robotx_msgs::GeographicPoint geopoint)
{
    point_with_area point;
    point.area = LatLonToUTMXY(geopoint.geographic_point.latitude,geopoint.geographic_point.longitude,
        0,point.point.x,point.point.y);
    point.node_id = geopoint.node_id;
    return point;
}

void coast_line_publisher::fix_callback_(const sensor_msgs::NavSatFixConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    double x,y;
    utm_area_buf_.push_back(LatLonToUTMXY(msg->latitude,msg->longitude,0,x,y));
    if(utm_area_buf_.size() == 1)
    {
        current_coast_lines_ = get_coast_lines_();
    }
    else if(utm_area_buf_[1] != utm_area_buf_[0])
    {
        current_coast_lines_ = get_coast_lines_();
    }
    coast_line_pub_.publish(current_coast_lines_);
    return;
}

void coast_line_publisher::publish_marker_()
{
    visualization_msgs::Marker marker;
    marker.type = marker.LINE_LIST;
    //marker.header.frame_id = 
    return;
}

void coast_line_publisher::map_callback_(const robotx_msgs::GeographicMapConstPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    world_frame_ = msg->header.frame_id;
    all_lines_ = msg->lines;
    std::vector<robotx_msgs::GeographicPoint> all_geo_points = msg->points;
    for(auto geo_point_itr = all_geo_points.begin(); geo_point_itr != all_geo_points.end(); geo_point_itr++)
    {
        all_points_.push_back(convert_geopoint_(*geo_point_itr));
    }
    utm_area_buf_.clear();
    return;
}

robotx_msgs::CoastLineArray coast_line_publisher::get_coast_lines_()
{
    robotx_msgs::CoastLineArray coast_lines;
    std::vector<point_with_area> points_in_area = filter_points_();
    ros::Time now = ros::Time::now();
    coast_lines.header.frame_id = world_frame_;
    coast_lines.header.stamp = now;
    for(auto line_itr = all_lines_.begin(); line_itr != all_lines_.end(); line_itr++)
    {
        boost::optional<point_with_area> start_point = query_point_(line_itr->start_node_id,points_in_area);
        boost::optional<point_with_area> end_point = query_point_(line_itr->end_node_id,points_in_area);
        if(start_point && end_point)
        {
            robotx_msgs::CoastLine line;
            line.header.frame_id = world_frame_;
            line.header.stamp = now; 
            line.start_point = start_point->point;
            line.end_point = end_point->point;
            coast_lines.coast_lines.push_back(line);
        }
    }
    return coast_lines;
}

std::vector<point_with_area> coast_line_publisher::filter_points_()
{
    std::vector<point_with_area> points_in_area_;
    if(utm_area_buf_.size() == 0)
    {
        return points_in_area_;
    }
    for(auto point_itr = all_points_.begin(); point_itr != all_points_.end(); point_itr++)
    {
        if(utm_area_buf_.size() == 1)
        {
            if(point_itr->area != utm_area_buf_[0])
            {
                points_in_area_.push_back(*point_itr);
            }
        }
        if(utm_area_buf_.size() == 2)
        {
            if(point_itr->area != utm_area_buf_[1])
            {
                points_in_area_.push_back(*point_itr);
            }
        }
    }
    return points_in_area_;
}