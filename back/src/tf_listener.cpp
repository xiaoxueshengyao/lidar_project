/***
 * Description:tf监听模块的实现
 * Author： Capta1n
 * Data: 0403
 * ****/


#include "tf_listener.hpp"
#include <Eigen/Geometry>

namespace lidar_project{
TFListener::TFListener(ros::NodeHandle& nh,std::string base_frame_id,std::string child_frame_id)
    :nh_(nh),base_frame_id(base_frame_id),child_frame_id(child_frame_id){

}

bool TFListener::LookupData(Eigen::Matrix4f& transform_matrix){
    try{
        tf::StampedTransform transform;
        listener_.lookupTransform(base_frame_id,child_frame_id,ros::Time(0),transform);
        //一个更底层的方法用于返回两个坐标系的变换。base->child
        //https://www.ncnynl.com/archives/201702/1308.html
        TransformToMatrix(transform,transform_matrix);
        return true;
        
    }
    catch(tf::TransformException &ex){
        return false;
    }

}

bool TFListener::TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix){

    //设置平移坐标
    Eigen::Translation3f tf_btol(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());

    double roll,pitch,yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw,pitch,roll);
    Eigen::AngleAxisf rot_x(roll,Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(pitch,Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(yaw,Eigen::Vector3f::UnitZ());

    //child 到base的矩阵
    transform_matrix = (tf_btol * rot_z * rot_y * rot_x).matrix();//这样也行？
    //参考Transform<float,3,Affine> T = Translation3f(p) * AngleAxisf(a,axis) * Scaling(s)
    return true;
}


}