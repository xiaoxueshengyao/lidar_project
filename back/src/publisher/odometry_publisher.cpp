#include "publisher/odometry_publisher.hpp"

namespace lidar_project{

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