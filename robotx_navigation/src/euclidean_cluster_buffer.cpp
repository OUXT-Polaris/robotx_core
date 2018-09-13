#include <euclidean_cluster_buffer.h>

euclidean_cluster_buffer::euclidean_cluster_buffer(double buffer_length) : _tf_listener(_tf_buffer), _buffer_length(buffer_length)
{
    
}

euclidean_cluster_buffer::~euclidean_cluster_buffer()
{
    
}

std::vector<cluster_data> euclidean_cluster_buffer::get_cluster_data()
{
    std::vector<cluster_data> ret;
    _mtx.lock();
    ros::Time now = ros::Time::now();
    std::vector<boost::shared_ptr<cluster_data> > new_buffer;
    for(auto cluster_itr = _buffer.begin(); cluster_itr != _buffer.end(); cluster_itr++)
    {
        boost::shared_ptr<cluster_data> cluster_ptr = *cluster_itr;
        if(now - _buffer_length < cluster_ptr->point.header.stamp)
        {
            new_buffer.push_back(*cluster_itr);
            cluster_data data(cluster_ptr->point, cluster_ptr->radius);
            ret.push_back(data);
        }
    }
    _buffer = new_buffer;
    _mtx.unlock();
    return ret;
}

void euclidean_cluster_buffer::add_cluster_data(cluster_data data)
{
    _mtx.lock();
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = _tf_buffer.lookupTransform("world", data.point.header.frame_id, data.point.header.stamp);
        geometry_msgs::PointStamped point = data.point;
        tf2::doTransform(point, point, transform_stamped);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }
    ros::Time now = ros::Time::now();
    std::vector<boost::shared_ptr<cluster_data> > new_buffer;
    new_buffer.push_back(boost::make_shared<cluster_data>(data.point,data.radius));
    for(auto cluster_itr = _buffer.begin(); cluster_itr != _buffer.end(); cluster_itr++)
    {
        boost::shared_ptr<cluster_data> cluster_ptr = *cluster_itr;
        if(now - _buffer_length < cluster_ptr->point.header.stamp)
        {
            new_buffer.push_back(*cluster_itr);
        }
    }
    _buffer = new_buffer;
    _mtx.unlock();
    return;
}