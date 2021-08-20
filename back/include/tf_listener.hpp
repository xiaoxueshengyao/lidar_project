/***
 * Description: tf监听模块，imu到lidar的转换,
 * Author: Capta1n
 * Data: 0403
 * ***/
#ifndef LIDAR_PROJECT_TF_LISTENER_HPP_
#define LIDAR_PROJECT_TF_LISTENER_HPP_


#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>



namespace lidar_project{
class TFListener{
  public:
    TFListener(ros::NodeHandle& nh,std::string base_frame_id, std::string child_frame_id);
    TFListener() = default;

    bool LookupData(Eigen::Matrix4f& transform_matrix);
  private:
    //把tf类型转换为eigen用于后续坐标转换
    bool TransformToMatrix(const tf::StampedTransform& transform,Eigen::Matrix4f& transform_matrix);

  private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id;
    std::string child_frame_id;
};


}

#endif