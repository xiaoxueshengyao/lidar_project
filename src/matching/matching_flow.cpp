/*****
 * Description: 地图匹配定位流程管理
 * Data: 1019
 * ****/

#include "matching/matching_flow.hpp"
#include "glog/logging.h"
#include "general_models/tools/global_path.h"

namespace lidar_project{
MatchingFlow::MatchingFlow(ros::NodeHandle& nh){
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh,"/synced_cloud",100000);
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh,"/synced_gnss",100000);

    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh,"/global_map","/map",100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh."local_map","/map",100);
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh,"/current_scan","/map",100);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh,"/laser_localization","/map","/lidar",100);
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map","/vehicle_link");

    matching_ptr_ = std::make_shared<Matching>();
}


bool MatchingFlow::Run(){
    if(matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()){
        CloudData::CloudPtr global_map_ptr(new CloudData::CloudPointT);
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);//获取全局地图并发布
    }

    if(matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
        local_map_pub_ptr_->Publish(matching_ptr_->GetLoaclMap());

    ReadData();
    while(HasData()){
        if(!ValidData())
            continue;
        if(UpdateMatching()){
            PublishData();
        }
    }
    return true;
}

bool MatchingFlow::ReadData(){
    cloud_sub_ptr_->ParaData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    return true;
}

/***确定有数据，确定初始化***/
bool MatchingFlow::HasData(){
    if(cloud_data_buff_.size() == 0)
        return false;
    if(matching_ptr_->HasInited())
        return true;
    if(gnss_data_buff_.size() == 0)
        return false;
    
    return true;
}


bool MatchingFlow::ValidData(){
    current_cloud_data_ = cloud_data_buff_.front();
    //初始化完成的话，把上次的数据就弹出了
    if(matching_ptr_->HasInited()){
        cloud_data_buff_.pop_front();
        gnss_data_buff_.clear();

        return true;
    }

    current_gnss_data_ = gnss_data_buff_.front();

    double diff_time = current_cloud_data_.time - current_gnss_data_.time;
    if(diff_time < -0.05 ){
        cloud_data_buff_.pop_front();
        return false;
    }
    if(diff_time > 0.05){
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;

}


/***没有初始化就出事后后更新，初始化了就直接更新位姿***/
bool MatchingFlow::UpdateMatching(){
    if(!matching_ptr_->HasInited()){
        matching_ptr_->SetGNSSPose(current_gnss_data_.pose);//如果没初始化，则第一帧gnss数据作为初始化
    }

    return matching_ptr_->Update(current_cloud_data_,laser_odometry_);


}

/**发布tf数据，历程及数据和对应点云数据***/
bool MatchingFlow::PublishData(){
    laser_tf_pub_ptr_->SendTransform(laser_odometry_,current_cloud_data_.time);
    laser_odom_pub_ptr_->Publish(laser_odometry_,current_cloud_data_.time);
    current_scan_pub_ptr_->Publish(matching_ptr_->GetCurrentScan());

    return true;
}







}