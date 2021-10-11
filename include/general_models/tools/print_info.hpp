/****
 * Description:打印信息
 * Data:1011
 * ***/

#ifndef PRINT_INFO_HPP_
#define PRINT_INFO_HPP_

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <pcl/common/eigen.h>


namespace lidar_projectr{
class PrintInfo{
    public:
        static void PrintPose(std::string head, Eigen::Matrix4f pose);//static的作用是只在该文件中能调用，其他的不行
};

}





#endif