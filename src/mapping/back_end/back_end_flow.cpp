/***
 * Description:后端流程管理
 * 功能———从里程计位姿中提取关键帧
 *       根据关键帧位姿、GNss约束和闭环检测约束来优化位姿
 * Input : 前段里程计位姿 + GNSS组合导航位姿 +　闭环检测相对位姿
 * Output: 优化后的位姿
 * Data:0610
 * 
 * 1018 前端使用aloam，对应话题需要更改，从构造函数入手
 * ***/


#include "mapping/back_end/back_end_flow.hpp"

#include "general_models/file_manager/file_manager.hpp"
#include "glog/logging.h"
#include "general_models/tools/global_path.h"


namespace lidar_project{

//构造函数，初始化私有变量
// BackEndFlow::BackEndFlow(ros::NodeHandle& nh){
BackEndFlow::BackEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic){
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh,"/synced_cloud",100000);
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh,cloud_topic,100000);
    // gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh,"synced_gnss",100000);
    // laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh,"laser_odom",100000);
    gnss_pose_sub_ptr_ = std::shared_ptr<OdometrySubscriber>(new OdometrySubscriber(nh,"/synced_gnss",100000));
    // laser_odom_sub_ptr_ = std::shared_ptr<OdometrySubscriber>(new OdometrySubscriber(nh,"/laser_odom",100000));
    laser_odom_sub_ptr_ = std::shared_ptr<OdometrySubscriber>(new OdometrySubscriber(nh,odom_topic,100000));
    loop_pose_sub_ptr_ = std::make_shared<LoopPoseSubscriber>(nh,"/loop_pose",100000);//回环数据订阅指针

    //发布优化后的位姿
    // transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh,"transformed_odom","/map","/lidar",100);
    // key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh,"/key_frame","map",100);
    transformed_odom_pub_ptr_ = std::shared_ptr<OdometryPublisher>(new OdometryPublisher(nh, "/transformed_odom", "/map", "/lidar", 100));
    key_frame_pub_ptr_ = std::shared_ptr<KeyFramePublisher>(new KeyFramePublisher(nh,"/key_frame", "/map",100));
    key_gnss_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh,"/key_gnss","/map",100);
    //优化后的关键帧位姿
    // key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh,"/optimized_key_frames","map",100);
    key_frames_pub_ptr_ = std::shared_ptr<KeyFramesPublisher>(new KeyFramesPublisher(nh,"/optimized_key_frames", "/map",100));

    // back_end_ptr_ = std::make_shared<BackEnd>();
    back_end_ptr_ = std::shared_ptr<BackEnd>(new BackEnd());

}


//流程管理，包括前段数据有效性判断，后端更新，发布相关数据
bool BackEndFlow::Run(){
    if(!ReadData())
        return false;
    MaybeInsertLoopPose();
    
    while(HasData())
    {
        if(!ValidData())
            continue;
        UpdateBackEnd();

        PublishData();
    }
    return true;

}

//更新优化
bool BackEndFlow::ForceOptimize(){
    back_end_ptr_->ForceOptimize();
    if(back_end_ptr_->HasNewOptimized()){
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);//更新后发布
    }
    return true;
}

//数据订阅后获取
bool BackEndFlow::ReadData(){
    cloud_sub_ptr_->ParaData(cloud_data_buff_); //都放到deque中
    gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);

    loop_pose_sub_ptr_->ParseData(loop_pose_data_buff_);

    return true;

}

//判断是否需要插入回环位姿
bool BackEndFlow::MaybeInsertLoopPose(){
    while(loop_pose_data_buff_.size()>0){
        back_end_ptr_->InsertLoopPose(loop_pose_data_buff_.front());
        loop_pose_data_buff_.pop_front();
    }
    return true;
}

//判断容器中是否有数据
bool BackEndFlow::HasData(){
    if(cloud_data_buff_.size() == 0)
        return false;
    if(gnss_pose_data_buff_.size() ==0)
        return false;
    if(laser_odom_data_buff_.size() == 0)
        return false;
    
    return true;
}


//验证数据有效性，主要是时间差不能太大
bool BackEndFlow::ValidData(){
    current_cloud_data_ = cloud_data_buff_.front();
    current_gnss_pose_data_ = gnss_pose_data_buff_.front();
    current_laser_pose_data_ = laser_odom_data_buff_.front();


    double diff_gnss_time = current_cloud_data_.time - current_gnss_pose_data_.time;
    double diff_laser_time = current_cloud_data_.time - current_laser_pose_data_.time;


    //如果时间差距过大，根据实际情况进行舍去
    if(diff_gnss_time < -0.05 || diff_laser_time < -0.05){
        cloud_data_buff_.pop_front();
        return false;
    }
    if(diff_gnss_time > 0.05){
        gnss_pose_data_buff_.pop_front();
        return false;
    }
    if(diff_laser_time > 0.05){
        laser_odom_data_buff_.pop_front();
        return false;
    }

    //如果正常，在保留了前面的正常数据后，pop，给后续使用
    cloud_data_buff_.pop_front();
    gnss_pose_data_buff_.pop_front();
    laser_odom_data_buff_.pop_front();

    return true;
}

//先把激光里程计数据的坐标转换到gnss的坐标系，做了对齐
bool BackEndFlow::UpdateBackEnd(){
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if(!odometry_inited){
        odometry_inited = true;
        odom_init_pose = current_gnss_pose_data_.pose * current_laser_pose_data_.pose.inverse();
    }
    current_laser_pose_data_.pose = odom_init_pose * current_laser_pose_data_.pose;
    
    return back_end_ptr_->Updata(current_cloud_data_, current_laser_pose_data_, current_gnss_pose_data_);


}


bool BackEndFlow::PublishData(){
    transformed_odom_pub_ptr_->Publish(current_laser_pose_data_.pose,current_laser_pose_data_.time);

    //有新关键帧时发布单个关键帧及其对应gnss
    if(back_end_ptr_->HasNewKeyFrame()){
        KeyFrame key_frame;

        back_end_ptr_->GetLatestKeyFrame(key_frame);
        key_frame_pub_ptr_->Publish(key_frame);

        back_end_ptr_->GetLatestKeyGNSS(key_frame);
        key_gnss_pub_ptr_->Publish(key_frame);
    }

    //做了一次优化后，把所有关键帧都发布一次
    if(back_end_ptr_->HasNewOptimized()){
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;




}

}