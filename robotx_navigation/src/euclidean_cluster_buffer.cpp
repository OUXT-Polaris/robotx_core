#include <euclidean_cluster_buffer.h>

euclidean_cluster_buffer::euclidean_cluster_buffer(double buffer_length, std::string map_frame) : _tf_listener(_tf_buffer), _buffer_length(buffer_length), _map_frame(map_frame)
{
    
}

euclidean_cluster_buffer::~euclidean_cluster_buffer()
{
    
}

std::vector<cluster_data> euclidean_cluster_buffer::get_cluster_data(std::string frame_name)
{
    std::vector<cluster_data> ret;
    geometry_msgs::TransformStamped transform_stamped;
    _mtx.lock();
    for(int i=0; i<_buffer.size(); i++)
    {
        try
        {
            geometry_msgs::PointStamped transformed_point;
            transform_stamped = _tf_buffer.lookupTransform(_map_frame, frame_name, _buffer[i]->point.header.stamp);
            tf2::doTransform(_buffer[i]->point, transformed_point, transform_stamped);
            ret.push_back(cluster_data(transformed_point,_buffer[i]->radius));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
        }
    }
    _mtx.unlock();
    return ret;
}

std::vector<cluster_data> euclidean_cluster_buffer::get_cluster_data()
{
    std::vector<cluster_data> ret;
    _mtx.lock();
    for(int i=0; i<_buffer.size(); i++)
    {
        ret.push_back(*_buffer[i]);
    }
    _mtx.unlock();
    return ret;
}

void euclidean_cluster_buffer::add_cluster_data(cluster_data data)
{
    _mtx.lock();
    geometry_msgs::TransformStamped transform_stamped;
    geometry_msgs::PointStamped point = data.point;
    try
    {
        transform_stamped = _tf_buffer.lookupTransform(_map_frame, data.point.header.frame_id, data.point.header.stamp);
        tf2::doTransform(point, point, transform_stamped);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
    }
    std::vector<boost::shared_ptr<cluster_data> > new_buffer;
    new_buffer.push_back(boost::make_shared<cluster_data>(point,data.radius));
    for(auto cluster_itr = _buffer.begin(); cluster_itr != _buffer.end(); cluster_itr++)
    {
        boost::shared_ptr<cluster_data> cluster_ptr = *cluster_itr;
        if(ros::Time::now() - _buffer_length < cluster_ptr->point.header.stamp)
        {
            new_buffer.push_back(*cluster_itr);
        }
    }
    _buffer = new_buffer;
    _mtx.unlock();
    return;
}