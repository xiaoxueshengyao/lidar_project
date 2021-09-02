/****
 * Des: 写个前段的流程，相当于任务管理，这样所有功能都放在类里，main中就创建个对象
 * Author: Capta1nY
 * Data:0415
 * 0601 继续添加速度信息
 * 0604 添加路径保存，便于评价里程计精度
 * ***/

#ifndef FRONT_END_FLOW_HPP_
#define FRONT_END_FLOW_HPP_


#include <ros/ros.h>
#include "subscriber/cloud_subscriber.hpp" //点云数据订阅

#include "publisher/odometry_publisher.hpp"  //数据发布

#include "front_end.hpp"       //前端内容


namespace lidar_project{
class FrontEndFlow{
    public:
      FrontEndFlow(ros::NodeHandle& nh);

      bool Run();//所有流程在这

    private:
      bool ReadData();
      bool HasData();
      bool ValidData();
      bool UpdateLaserOdometry();
      bool PublishData();



    private:
      //数据订阅
      std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  
      //里程计发布
      std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;

      //前端
      std::shared_ptr<FrontEnd> front_end_ptr_;

      std::deque<CloudData> cloud_data_buff_;
      
      CloudData current_cloud_data_;


      Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

};


}






#endif