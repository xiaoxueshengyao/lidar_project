/***
 * Description:回环流程管理
 * Data:1012
 * ***/

#include "mapping/loop_closing/loop_closing_flow.hpp"
#include "general_models/tools/global_path.h"
#include "glog/logging.h"

namespace lidar_project{

//构造函数，给定义的私有变量赋值，就是几个指针
LoopClosingFlow::LoopClosingFlow(ros::NodeHandle& nh){
    //订阅指针赋值
    key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh,"/key_frame",100000);
    key_gnss_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh,"/key_gnss",100000);

    //回环位姿发布指针
    loop_pose_pub_ptr_ = std::make_shared<LoopPosePublisher>(nh, "/loop_pose", "/map", 100);

    //回环检测指针
    loop_closing_ptr_ = std::make_shared<LoopClosing>();

}



bool LoopClosingFlow::Run(){
    if(!ReadData())
        return false;
    while(HasData())
    {
        if(!ValidData())
            continue;
        loop_closing_ptr_->Update(current_key_frame_,current_key_gnss_);

        PublishData();
    }

    return true;
    
}

bool LoopClosingFlow::ReadData(){
    key_frame_sub_ptr_->ParseData(key_frame_buff_);
    key_gnss_sub_ptr_->ParseData(key_gnss_buff_);

    return true;
}

bool LoopClosingFlow::HasData(){
    if(key_frame_buff_.size() == 0)
        return false;
    if(key_gnss_buff_.size() == 0)
        return false;

    return true;
}


bool LoopClosingFlow::ValidData(){
    current_key_frame_ = key_frame_buff_.front();
    current_key_gnss_ = key_gnss_buff_.front();

    double diff_gnss_time = current_key_frame_.time - current_key_gnss_.time;

    if(diff_gnss_time < -0.05){
        key_frame_buff_.pop_front();
        return false;
    }

    if(diff_gnss_time > 0.05){
        key_gnss_buff_.pop_front();
        return false;
    }

    key_frame_buff_.pop_front();
    key_gnss_buff_.pop_front();

    return true;
}


bool LoopClosingFlow::PublishData(){
    if(loop_closing_ptr_->HasNewLoopPose())
        loop_pose_pub_ptr_->Publish(loop_closing_ptr_->GetCurrentLoopPose());

    return true;
}





}