/***
 * @Des:点云畸变补偿
 * @Aut:Capta1nY
 * @Dat:0608
 * ***/

#include "general_models/undistorted/undistorted.hpp"
#include "glog/logging.h"

namespace lidar_project
{
//主要是获得角速度和线速度信息，用于后续转换
void DistortionAdjust::SetMotionInfo(float scan_period,VelocityData velocity_data){
    std::cout<<"Ready to get Velocity Infomation"<<std::endl;
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x,velocity_data.linear_velocity.y,velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x,velocity_data.angular_velocity.y,velocity_data.angular_velocity.z;
    std::cout<<"Got Velocity Infomation"<<std::endl;
}

bool DistortionAdjust::AdjustCloud(CloudData::CloudPtr& input_cloud_ptr, CloudData::CloudPtr& output_cloud_ptr){
    CloudData::CloudPtr original_cloud_ptr(new CloudData::CloudPointT(*input_cloud_ptr));
    output_cloud_ptr->points.clear();

    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;//5°内误差较小，直接保留
    float start_orientation = atan2(original_cloud_ptr->points[0].y,original_cloud_ptr->points[0].x);//atan2更厉害，分子是0也可计算

    Eigen::AngleAxisf t_V(start_orientation,Eigen::Vector3f::UnitZ());//轴角
    Eigen::Matrix3f rotate_matrix = t_V.matrix();//旋转矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block(0,0,3,3) = rotate_matrix.inverse();//到这里只有旋转信息
    //这样搞相当于把第一个点云的初始位置转到了正前方，这样余下的点直接求反正切就是角度差，不用再与起始位置的角度作差
    pcl::transformPointCloud(*original_cloud_ptr,*original_cloud_ptr,transform_matrix);
    //线、角速度也转换过去
    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    std::cout<<"Ready to transform all points"<<std::endl;
    for(size_t point_index=1; point_index < original_cloud_ptr->points.size(); point_index++){
        float orientation = atan2(original_cloud_ptr->points[point_index].y, original_cloud_ptr->points[point_index].x);
        if(orientation < 0.0){
            orientation += 2.0 * M_PI;
        }
        if(orientation < delete_space || 2.0 * M_PI - orientation < delete_space){
            continue;//5°以内的不作处理，这个根据实际情况改
        }
        
        //计算当前时间，其实是两帧点云的中间时刻，bag里的点云时刻是该帧点云起始和终止时刻的平均值，所以这里减去半周期，统一到中间时刻
        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_/2.0;
        Eigen::Vector3f original_point(original_cloud_ptr->points[point_index].x,
                                       original_cloud_ptr->points[point_index].y,
                                       original_cloud_ptr->points[point_index].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);//计算当前点的旋转矩阵
        Eigen::Vector3f rotated_point = current_matrix * original_point;
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;//旋转+平移
        CloudData::PointT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);

    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());//????
    return true;

}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time){
    Eigen::Vector3f angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;
    return t_V.matrix();
}
    
} // namespace lidar_project

