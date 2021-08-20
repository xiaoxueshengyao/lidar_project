/***
 * Des:主要为点云和里程计数据的发布，放在main中降低程序可读性，写成类
 * Author: Capta1nY
 * Data:0415
 * ***/

#ifndef DATA_PUBLISHER_HPP_
#define DATA_PUBLISHER_HPP_


#include "kitti_data/imu_data.hpp"
#include "kitti_data/gnss_data.hpp"
#include "kitti_data/cloud_data.hpp"
#include "kitti_data/velocity_data.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>

namespace lidar_project{
//点云发布
class CloudPublisher{
    public:
      CloudPublisher(ros::NodeHandle& nh, std::string topic_name,size_t buff_size,std::string frame_id);
      CloudPublisher() = default;
      void Publish(CloudData::CloudPtr cloud_ptr);


    private:
      ros::NodeHandle nh_;
      ros::Publisher publisher_;
      std::string frame_id_;


};

//里程计发布
class OdometryPublisher{
    public:
      OdometryPublisher(ros::NodeHandle& nh, std::string topic_name,std::string base_frame_id,
                        std::string child_frame_id, int buff_size);
      OdometryPublisher() = default;
      void Publish(const Eigen::Matrix4f& tansform_matrix);

    private:
      ros::NodeHandle nh_;
      ros::Publisher publisher_;
      nav_msgs::Odometry odometry_;  
};





}



#endif