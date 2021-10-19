/***
 * Description:后端流程管理
 * 功能———从里程计位姿中提取关键帧
 *       根据关键帧位姿、GNss约束和闭环检测约束来优化位姿
 * Input : 前段里程计位姿 + GNSS组合导航位姿 +　闭环检测相对位姿
 * Output: 优化后的位姿
 * Data:0610
 * ***/

#ifndef BACK_END_FLOW_HPP_
#define BACK_END_FLOW_HPP_


#include <ros/ros.h>

#include "mapping/back_end/back_end.hpp"

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"

#include "publisher/odometry_publisher.hpp"
#include "publisher/key_frame_publisher.hpp"
#include "publisher/key_frames_publisher.hpp"

// 回环位姿订阅
#include "subscriber/loop_pose_subscriber.hpp"



namespace lidar_project{
class BackEndFlow{
    public:
        // BackEndFlow(ros::NodeHandle& nh);
        BackEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic);
        bool Run();

        bool ForceOptimize();//调用优化

    private:
        bool ReadData();
        bool HasData();
        bool ValidData();
        bool UpdateBackEnd();
        // bool SaveTrajectory();
        bool PublishData();

        bool MaybeInsertLoopPose();//回环添加到后端

    private:
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;

        std::shared_ptr<LoopPoseSubscriber> loop_pose_sub_ptr_;//回环新增

        std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
        
        std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
        std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
        std::shared_ptr<BackEnd> back_end_ptr_;

        std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;//回环新增

        std::deque<CloudData> cloud_data_buff_;
        std::deque<PoseData,Eigen::aligned_allocator<PoseData>> gnss_pose_data_buff_;//Eigen内存对齐与std标准容器
        std::deque<PoseData,Eigen::aligned_allocator<PoseData>> laser_odom_data_buff_;
        std::deque<LoopPose> loop_pose_data_buff_;

        PoseData current_gnss_pose_data_;
        PoseData current_laser_pose_data_;
        CloudData current_cloud_data_;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




}



#endif