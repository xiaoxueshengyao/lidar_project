/***
 * Des:Realization of front end flow
 * Author: Capta1nY
 * 0416
 * 0506 时间同步，以雷达数据为核心做差值
 * ***/

#include "mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "general_models/file_manager/file_manager.hpp"


namespace lidar_project{

//构造函数初始化数据订阅、发布以及前端指针
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh){
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh,"/synced_cloud",100000);
 

    // laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh,"laser_odom","map","lidar",100);
    laser_odom_pub_ptr_ = std::shared_ptr<OdometryPublisher>(new OdometryPublisher(nh,"/laser_odom","/map","/lidar",100));


    // front_end_ptr_ = std::make_shared<FrontEnd>();
    front_end_ptr_ = std::shared_ptr<FrontEnd>(new FrontEnd());

    std::cout<<" Initilized all data "<<std::endl;


}

bool FrontEndFlow::Run(){

    if(!ReadData())   
      return false;

    while (HasData())
    {
        if(!ValidData()){
            continue;
        }
        if(UpdateLaserOdometry()){
            PublishData();
        }
    }

    return true;
    
}



//读入数据
bool FrontEndFlow::ReadData(){
    cloud_sub_ptr_->ParaData(cloud_data_buff_);
    return true;
}


//判断是否读入数据
bool FrontEndFlow::HasData(){

    return cloud_data_buff_.size()>0;
}


//判断是否是一帧数据(gps,imu,cloud)
//之前数据预处理的时候已经做过了，前端部分只负责计算位姿
bool FrontEndFlow::ValidData(){
    
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();
    return true;

}


//更新激光里程计信息
bool FrontEndFlow::UpdateLaserOdometry(){


    static bool front_end_pose_inited = false;//使用静态变量，类的
    if(!front_end_pose_inited){
        front_end_pose_inited = true;
        front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
        return front_end_ptr_->Update(current_cloud_data_,laser_odometry_);

    }
    if(front_end_ptr_->Update(current_cloud_data_,laser_odometry_))  return true;
    else return false;
   
}

bool FrontEndFlow::PublishData(){
    laser_odom_pub_ptr_->Publish(laser_odometry_,current_cloud_data_.time);//这里的时间忘了改，所以后端验证数据的时候一直过不去

    return true;
}



}