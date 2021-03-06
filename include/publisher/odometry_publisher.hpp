/***
 * 0604参考kitti_data把数据发布也分开
 * ***/

#ifndef ODOMETRY_PUBLISHER_HPP_
#define ODOMETRY_PUBLISHER_HPP_

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <string>


namespace lidar_project{

//里程计发布
class OdometryPublisher{
    public:
      OdometryPublisher(ros::NodeHandle& nh, 
                        std::string topic_name,
                        std::string base_frame_id,
                        std::string child_frame_id, 
                        int buff_size);
      OdometryPublisher() = default;
      
      void Publish(const Eigen::Matrix4f& transform_matrix);
      void Publish(const Eigen::Matrix4f& transform_matrix, double time);
      bool HasSubscribers();

    private:
      void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

    private:
      ros::NodeHandle nh_;
      ros::Publisher publisher_;
      nav_msgs::Odometry odometry_;  
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};





}





#endif