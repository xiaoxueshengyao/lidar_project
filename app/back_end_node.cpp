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

using namespace lidar_project;

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;//记录日志的同时输出到stderr
    FLAGS_colorlogtostderr = 1;//将彩色日志输出到stderr
    
    ros::init(argc,argv,"back_end_node");
    ros::NodeHandle nh;

    //创建后端流程管理指针
    // std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);
    std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::shared_ptr<BackEndFlow>(new BackEndFlow(nh));
    
    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        back_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}