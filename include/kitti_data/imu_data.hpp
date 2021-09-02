/***
 * 0603把几种数据分开不同文件，尝试解决同步时无法完成的问题
 * Capta1nY
 * 0827，对于Eigen的错误提示，使用EIGEN_MAKE_ALIGNED_OPERATOR_NEW这个宏来搞内存对齐
 * Assertion `(internal::UIntPtr(eigen_unaligned_array_assert_workaround_
 * gcc47(array)) & (31)) == 0 && "this assertion is explained here:
 * ***/ 


#ifndef IMU_DATA_HPP_
#define IMU_DATA_HPP_


#include <Eigen/Dense>
#include <deque>
#include <cmath>
#include "Eigen/StdDeque"

namespace lidar_project{

class IMUData{

  public:
    class Orientation{
      public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;

      public:
        //四元数归一化
        void Normalize(){
          double norm = sqrt(pow(x,2.0) + pow(y,2.0) + pow(z,2.0) + pow(w,2.0));
          x /= norm;
          y /= norm;
          z /= norm;
          w /= norm;
        }
    };

    struct AngularVelocity{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct LinearAcceleration{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
  
  double time=0.0;
  Orientation orientation;
  AngularVelocity angular_velocity;
  LinearAcceleration linear_acceleration;

  public:
    Eigen::Matrix3f GetRotateMat();

    static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time);

  public:
    //内存对齐方案
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
};
}
#endif


