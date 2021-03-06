/***
 * Description:可视化显示流程管理
 * Data:0615
 * ****/
#include "mapping/viewer/viewer_flow.hpp"
#include "glog/logging.h"


namespace lidar_project
{
//构造函数，各个指针的初始化
ViewerFlow::ViewerFlow(ros::NodeHandle& nh, std::string  cloud_topic){
    //数据订阅
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh,"/synced_cloud",100000);
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh,cloud_topic,100000);
    key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh,"/key_frame",100000);//多了个s成另外一个变量了
    transformed_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh,"/transformed_odom",100000);
    optimized_key_frames_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(nh,"/optimized_key_frames",100000);

    //数据发布
    current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh,"/current_scan","/map",100);
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh,"/global_map","/map",100);
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh,"/local_map","/map",100);
    optimized_odom_pub_ptr_ = std::shared_ptr<OdometryPublisher>(new OdometryPublisher(nh,"/optimized_odom","/map","/lidar",100));

    //显示
    viewer_ptr_ = std::shared_ptr<Viewer>(new Viewer());
}   

//流程控制
bool ViewerFlow::Run(){
    if(!ReadData())
        return false;
    
    while(HasData()){//读入订阅的里程计和点云数据
        std::cout<<"Viewer node has got Data>>>>>>>>>>>>>>>>>>>"<<std::endl;
        if(ValidData()){//这里去掉！
            //validata()把位姿和点云数据验证有效性，时间差较小
            viewer_ptr_->UpdateWithNewKeyFrame(key_frame_buff_,current_transformed_odom_,current_cloud_data_);
            PublishLocalData();//发布数据
 
        }
        //     continue;
        // if(UpdateViewer()){
        //     PublishData();
        // }
    }
    if(optimized_key_frames_.size()>0){
        viewer_ptr_->UpdateWithOptimizedKeyFrames(optimized_key_frames_);
        PublishGlobalData();
    }

    return true;
}

//数据读取
bool ViewerFlow::ReadData(){
    cloud_sub_ptr_->ParaData(cloud_data_buff_);
    transformed_odom_sub_ptr_->ParseData(transformed_odom_buff_);
    key_frame_sub_ptr_->ParseData(key_frame_buff_);
    optimized_key_frames_sub_ptr_->ParseData(optimized_key_frames_);

    return true;
}

bool ViewerFlow::HasData(){
    if(cloud_data_buff_.size() == 0){
        return false;
    }
    if(transformed_odom_buff_.size() == 0){
        return false;
    }

    return true;
}


bool ViewerFlow::ValidData(){
    current_cloud_data_ = cloud_data_buff_.front();
    current_transformed_odom_ = transformed_odom_buff_.front();//这里的等号有问题

    double diff_odom_time = current_cloud_data_.time - current_transformed_odom_.time;
    if(diff_odom_time < -0.05){
        cloud_data_buff_.pop_front();
        return false;
    }
    if(diff_odom_time > 0.05){
        transformed_odom_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    transformed_odom_buff_.pop_front();
    std::cout<<"Viewer node valid data"<<std::endl;
    return true;

}

//发布全局地图数据
bool ViewerFlow::PublishGlobalData(){
    if(viewer_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()){
        CloudData::CloudPtr cloud_ptr(new CloudData::CloudPointT());
        viewer_ptr_->GetGlobalMap(cloud_ptr);
        global_map_pub_ptr_->Publish(cloud_ptr);
    }

    return true;
}

//发布局部地图数据
bool ViewerFlow::PublishLocalData(){
    //获取当前位姿和点云数据
    optimized_odom_pub_ptr_->Publish(viewer_ptr_->GetCurrentPose());
    current_scan_pub_ptr_->Publish(viewer_ptr_->GetCurrentScan());
    
    //如果有新局部地图更新并且有订阅者则发布
    if(viewer_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()){
        CloudData::CloudPtr cloud_ptr(new CloudData::CloudPointT());
        viewer_ptr_->GetLocalMap(cloud_ptr);
        local_map_pub_ptr_->Publish(cloud_ptr);
    }
    return true;
}





// bool ViewerFlow::UpdateViewer(){
//     return viewer_ptr_->Update(key_frame_buff_,
//                                optimized_key_frames_,
//                                current_transformed_odom_,
//                                current_cloud_data_);

// }


// bool ViewerFlow::PublishData(){
//     optimized_odom_pub_ptr_->Publish(viewer_ptr_->GetCurrentPose());
//     current_scan_pub_ptr_->Publish(viewer_ptr_->GetCurrentScan());

//     if(viewer_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers()){
//         CloudData::CloudPtr cloud_ptr(new CloudData::CloudPointT);
//         viewer_ptr_->GetLocalMap(cloud_ptr);
//         local_map_pub_ptr_->Publish(cloud_ptr);
//     }

//     if(viewer_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers()){
//         CloudData::CloudPtr cloud_ptr(new CloudData::CloudPointT);
//         viewer_ptr_->GetGlobalMap(cloud_ptr);
//         global_map_pub_ptr_->Publish(cloud_ptr);
//     }

//     std::cout<<"viewer node Publish data"<<std::endl;
//     return true;


// }

bool ViewerFlow::SaveMap(){
    return viewer_ptr_->SaveMap();
}

} // namespace lidar_project

