//所有的类和其实现分开放在两个文件，不然会出现重复定义或者第一次在此定义的错误
//本来想偷个懒，结果费时间搞这个

#include "data_publisher.hpp"

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

OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh, std::string topic_name,std::string base_frame_id,
                        std::string child_frame_id, int buff_size)
    :nh_(nh){
    publisher_ = nh.advertise<nav_msgs::Odometry>(topic_name,buff_size);
    odometry_.header.frame_id = base_frame_id;
    odometry_.child_frame_id = child_frame_id;

}

void OdometryPublisher::Publish(const Eigen::Matrix4f& trasform_matrix){
    odometry_.header.stamp = ros::Time::now();
    //translation
    odometry_.pose.pose.position.x = trasform_matrix(0,3);
    odometry_.pose.pose.position.y = trasform_matrix(1,3);
    odometry_.pose.pose.position.z = trasform_matrix(2,3);

    //Rotation
    Eigen::Matrix3f R = trasform_matrix.block(0,0,3,3).matrix();//注意这里给四元数赋值的方式
    Eigen::Quaternionf q(R);
    odometry_.pose.pose.orientation.w = q.w();
    odometry_.pose.pose.orientation.x = q.x();
    odometry_.pose.pose.orientation.y = q.y();
    odometry_.pose.pose.orientation.z = q.z();
    publisher_.publish(odometry_);

}



}