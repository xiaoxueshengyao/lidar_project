#include "publisher/cloud_publisher.hpp"

namespace lidar_project{

CloudPublisher::CloudPublisher(ros::NodeHandle& nh, std::string topic_name,
                               size_t buff_size,std::string frame_id)
    :nh_(nh),frame_id_(frame_id){
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name,buff_size);
    
}

//就是把PCL点云数据转化为sensormsg
void CloudPublisher::Publish(CloudData::CloudPtr cloud_ptr){
    sensor_msgs::PointCloud2Ptr cloud_ptr_out(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr,*cloud_ptr_out);
    cloud_ptr_out->header.stamp = ros::Time::now();
    cloud_ptr_out->header.frame_id = frame_id_;
    publisher_.publish(cloud_ptr_out);
}


}//cloud data publisher