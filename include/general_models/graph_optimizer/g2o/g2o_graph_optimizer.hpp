/***
 * Des:后端优化的g2o接口，继承于图优化
 * Capta1nY
 * 0927
 * **/

#ifndef G2O_GRAPH_OPTIMIZER_HPP_
#define G2O_GRAPH_OPTIMIZER_HPP_

#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "general_models/graph_optimizer/interface_graph_optimizer.hpp"
#include "general_models/graph_optimizer/g2o/edge/edge_se3_priorquat.hpp"
#include "general_models/graph_optimizer/g2o/edge/edge_se3_priorxyz.hpp"

namespace g2o{
class VertexSE3;
class VertexPlane;
class VertexPointXYZ;
class EdgeSE3;
class EdgeSE3Plane;
class EdgeSE3PointXYZ;
class EdgeSE3PriorXY;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
} // namespace g2o

G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

//继承自优化接口类，接口中的函数基本都是纯虚函数，在继承后需要一一实现，关键字写override，不能再“=0”
namespace lidar_project{
class G2oGraphOptimizer : public InterfaceGraphOptimizer{
    public:
        G2oGraphOptimizer(const std::string& solver_type = "lm_var");//构造函数，求解器类型
        bool Optimize() override; //优化
        bool GetOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) override; //优化后位姿
        int GetNodeNum() override;
        //添加节点、边、鲁棒核
        void SetEdgeRobustKernel(std::string robust_kernel_name,double robust_kernel_size) override;
        void AddSe3Node(const Eigen::Isometry3d& pose, bool need_fix) override;//第一个点讨论是否fix
        void AddSe3Edge(int vertex_index1,
                        int vertex_index2,
                        const Eigen::Isometry3d& relative_pose,//帧间相对位姿
                        const Eigen::VectorXd noise) override;//信息矩阵，权重
        void AddSe3PriorXYZEdge(int se3_vertex_index,
                                const Eigen::Vector3d& xyz,
                                Eigen::VectorXd noise) override;
        void AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                       const Eigen::Quaterniond& quat,//姿态作为先验边
                                       Eigen::VectorXd noise) override;

    private:
        Eigen::MatrixXd CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise);//se3边信息矩阵
        Eigen::MatrixXd CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise);//姿态边信息矩阵
        Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);//对角阵
        void AddRobustKernel(g2o::OptimizableGraph::Edge* dege, const std::string& kernel_type, double kernel_size);

    private:
        g2o::RobustKernelFactory* robust_kernel_factory_;
        std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;

        std::string robust_kernel_name_;
        double robust_kernel_size_;
        bool need_robust_kernel_ = false;


};

}




#endif