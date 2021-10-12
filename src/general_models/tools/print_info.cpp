/****
 * Description:打印信息
 * Data:1011
 * ***/

#include "general_models/tools/print_info.hpp"
#include "glog/logging.h"

namespace  lidar_project
{
/***
 * Input:head,pose
 * Output:x,y,z,r,p,y
 * ***/
void PrintPose(std::string head, Eigen::Matrix4f pose){
    Eigen::Affine3f aff_pose;
    aff_pose.matrix() = pose;
    float x,y,z,roll,pitch,yaw;
    pcl::getTranslationAndEulerAngles(aff_pose,x,y,z,roll,pitch,yaw);//输入位姿矩阵，得到各个分量
    std::cout<<head<<"\n"<<x<<"\t"<<y<<"\t"<<z<<"\t"<<roll*180/M_PI<<"\t"<<pitch*180/M_PI<<"\t"<<yaw*180/M_PI<<std::endl;
}
} // namespace  lidar_project
