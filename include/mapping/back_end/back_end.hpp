/***
 * Description:后端模块
 * 功能———从里程计位姿中提取关键帧
 *       根据关键帧位姿、GNss约束和闭环检测约束来优化位姿
 * Input : 前段里程计位姿 + GNSS组合导航位姿 +　闭环检测相对位姿
 * Output: 优化后的位姿
 * Data: 0608
 * ***/

#ifndef BACK_END_HPP_
#define BACK_END_HPP_

#include "kitti_data/cloud_data.hpp"
#include "kitti_data/pose_data.hpp"
#include "kitti_data/key_frame.hpp"

#include "kitti_data/loop_pose.hpp"

#include <string>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "general_models/graph_optimizer/g2o/g2o_graph_optimizer.hpp"

namespace lidar_project{
class BackEnd{
    public:
        BackEnd();

        bool ForceOptimize();//强制更新
        bool InsertLoopPose(const LoopPose& loop_pose);

        bool Updata(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose);//更新位姿
        void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);
        bool HasNewKeyFrame();
        bool HasNewOptimized();
        void GetLatestKeyFrame(KeyFrame& key_frame);
        void GetLatestKeyGNSS(KeyFrame& key_frame);//回环新增，获取关键帧对应gnss数据

    private:
        bool InitWithConfig();                                      //关键帧、优化等参数选择
        bool InitParam(const YAML::Node& config_node);              //关键帧+优化条件
        bool InitDataPath(const YAML::Node& config_node);           //数据路径,轨迹等
        
        bool InitGraphOptimizer(const YAML::Node& config_node);     //后端优化参数读取

        void ResetParam();
        // bool SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose);//回环去掉
        bool MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_data);//回环添加gnss
        bool MaybeOptimized();

        bool AddNodeAndEdge(const PoseData& gnss_data);//添加节点和边

        //回环新增函数
        bool SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose);
        bool SaveOptimizedPose();


    private:
        std::string key_frames_path_ = "" ;
        std::string trajectory_path_ = "";
        
        std::ofstream ground_truth_ofs_;
        std::ofstream laser_odom_ofs_;
        std::ofstream optimized_pose_ofs_;//回环新增

        float key_frame_distance_ = 2.0;
        // int optimize_step_with_none_ = 100;             
        // int optimize_step_with_gnss_ = 100;             //带gnss数据的优化
        // int optimize_step_with_loop_ = 10;              //带回环的优化，优化阈值

        bool has_new_key_frame_ = false;
        bool has_new_optimized_ = false;
        
        // Eigen::Matrix4f last_key_pose_ = Eigen::Matrix4f::Identity();
        // KeyFrame latest_key_frame_;
        KeyFrame current_key_frame_;
        std::deque<KeyFrame> key_frames_deque_;
        KeyFrame current_key_gnss_;
        std::deque<Eigen::Matrix4f> optimized_pose_;


        //优化相关,优化器
        std::shared_ptr<InterfaceGraphOptimizer> graph_optimizer_ptr_;
        //嵌套类，私有，可以直接访问外围类的静态成员、类型名、枚举值
        class GraphOptimizerConfig{
            public:
                GraphOptimizerConfig(){
                    odom_edge_noise.resize(6);
                    close_loop_noise.resize(6);
                    gnss_noise.resize(3);
                }
            
            public:
                bool use_gnss = true;
                bool use_loop_close = false;

                //踊跃信息矩阵的计算
                Eigen::VectorXd odom_edge_noise;
                Eigen::VectorXd close_loop_noise;
                Eigen::VectorXd gnss_noise;

                int optimize_step_with_key_frame = 100;
                int optimize_step_with_gnss = 100;
                int optimize_step_with_loop = 10;
        };
        GraphOptimizerConfig graph_optimizer_config_;

        int new_gnss_cnt_ = 0;
        int new_loop_cnt_ = 0;
        int new_key_frame_cnt_ = 0;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};





}




#endif