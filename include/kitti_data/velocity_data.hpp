/***
 * 0603把几种数据分开不同文件，尝试解决同步时无法完成的问题
 * Capta1nY
 * ***/ 

#ifndef VELOCITY_DATA_HPP_
#define VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>
#include <eigen3/Eigen/StdDeque>

namespace lidar_project{
//速度（线、角）数据
class VelocityData{
  
  public:
    struct LinearVelocity{
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity{
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

  public:
    //同步
    static bool SyncData(std::deque<VelocityData>& UnsyncedData,
                         std::deque<VelocityData>& SyncedData,
                         double sync_time);
    
    //把imu的速度转换到雷达坐标系
    void TransformCoordinate(Eigen::Matrix4f& transform_matrix);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
#endif