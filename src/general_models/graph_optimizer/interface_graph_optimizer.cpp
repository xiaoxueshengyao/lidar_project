/***
 * Des:图优化接口实现，因为都是虚函数，就只有一个需要实现                                                       
 * 0927
 * ***/
#include "general_models/graph_optimizer/interface_graph_optimizer.hpp"


namespace lidar_project{
void InterfaceGraphOptimizer::SetMaxIterationNum(int max_iteration_num){
    max_iteration_num_ = max_iteration_num;
}
}