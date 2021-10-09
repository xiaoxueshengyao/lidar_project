/****
 * Description: 后端图优化节点，TODO
 * Task1--接收前端里程计和GNSS组合导航数据，按时间对齐后放在txt，便与后续评测里程计精度
 * Task2--从里程计位置中识别关键帧，并把关键帧位姿和对应的GNSS位姿数据publish
 * Task3--发送历史帧系列位姿，后续图优化使用，每次优化后把优化后的位姿读取出来，发送给其他模块使用
 * Data:0825
 * ****/

#include "mapping/back_end/back_end_flow.hpp"
#include "general_models/file_manager/file_manager.hpp"     //文件管理
#include "glog/logging.h"

#include "lidar_project/optimizeMap.h"
using namespace lidar_project;

std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request& request, optimizeMap::Response& response){
    _need_optimize_map = true;
    response.succeed = true;
    return response.succeed;
}

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/jingwan/lslidar_ws/src/lidar_project/Log";
    FLAGS_alsologtostderr = 1;//记录日志的同时输出到stderr

    
    ros::init(argc,argv,"back_end_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("optimize_map",optimize_map_callback);
    _back_end_flow_ptr = std::shared_ptr<BackEndFlow>(new BackEndFlow(nh));

    //创建后端流程管理指针
    // std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);
    // std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::shared_ptr<BackEndFlow>(new BackEndFlow(nh));
    
    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        _back_end_flow_ptr->Run();

        //优化
        if(_need_optimize_map){
            _back_end_flow_ptr->ForceOptimize();
            _need_optimize_map = false;
        }

        rate.sleep();
    }

    return 0;
}