/****
 * Des:优化接口，供给后端调用，目前采用g2o作为后端优化器
 * 只提供接口，可以用其他优化库进行实现
 * Capta1nY
 * 0926
 * ***/

#ifndef INTERFACE_GRAPH_OPTIMIZER_HPP
#define INTERFACE_GRAPH_OPTIMIZER_HPP

#include <string>
#include <deque>
#include <Eigen/Dense>>


namesapce lidar_project{
class InterfaceGraphOptimizer{
    public:
        virtual ~InterfaceGraphOptimizer(){}
        virtual bool Optimize() = 0;//纯虚函数，优化函数
        virtual bool GetOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) = 0;//输入输出数据
        virtual int GetNodeNum() = 0;
        //类似于g2o优化中的添加节点、边、鲁邦核函数
        virtual void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) = 0;
        virtual void AddSe3Node(const Eigen::Isometry3d& pose, bool need_fix) = 0;//第一个点讨论是否fix
        virtual void AddSe3Edge(int vertex_index1,
                                int vertex_index2,
                                const Eigen::Isometry3d& relative_pose,//帧间相对位姿
                                const Eigen::VectorXd noise) = 0;//信息矩阵，权重
        virtual void AddSe3PriorXYZEdge(int se3_vertex_index,
                                        const Eigen::Vector3d& xyz,
                                        Eigen::VectorXd noise) = 0;
        virtual void AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                               const Eigen::Quaterniond& quat,//姿态作为先验边
                                               Eigen::VectorXd noise) = 0;
        //设置优化迭代次数
        void SetMaxIterationNum(int max_iteration_num);

    protected:
        int max_iteration_num_ = 512;
};
}






#endif