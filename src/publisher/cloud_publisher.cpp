#include "publisher/cloud_publisher.hpp"

namespace lidar_project{

CloudPublisher::CloudPublisher(ros::NodeHandle& nh, 
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh),frame_id_(frame_id){
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name,buff_size);
    
}


void CloudPublisher::Publish(CloudData::CloudPtr& cloud_ptr, double time){
    ros::Time ros_time((float)time);
    PublishData(cloud_ptr,ros_time);

}

void CloudPublisher::Publish(CloudData::CloudPtr& cloud_ptr){
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr,time);

}

bool CloudPublisher::HasSubscribers(){
    return publisher_.getNumSubscribers() != 0;
}

//就是把PCL点云数据转化为sensormsg
void CloudPublisher::PublishData(CloudData::CloudPtr& cloud_ptr_input,ros::Time time){
    sensor_msgs::PointCloud2Ptr cloud_ptr_out(new sensor_msgs::PointCloud2());
    cloud_ptr_input->width = cloud_ptr_input->points.size();
    cloud_ptr_input->height = 1;
    pcl::toROSMsg(*cloud_ptr_input,*cloud_ptr_out);//转成ros数据

    cloud_ptr_out->header.stamp = time;
    cloud_ptr_out->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_out);
}


}//cloud data publisher