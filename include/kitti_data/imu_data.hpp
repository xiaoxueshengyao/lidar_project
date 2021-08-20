/***
 * 0603把几种数据分开不同文件，尝试解决同步时无法完成的问题
 * Capta1nY
 * ***/ 


#ifndef IMU_DATA_HPP_
#define IMU_DATA_HPP_


#include <Eigen/Dense>
#include <deque>
#include <cmath>

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

    
};
}
#endif


