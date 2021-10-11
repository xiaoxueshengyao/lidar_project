/****
 * Des:g2o优化接口实现
 * 0927
 * ***/

#include "general_models/graph_optimizer/g2o/g2o_graph_optimizer.hpp"
#include "glog/logging.h"
#include "general_models/tools/tic_toc.hpp"

namespace lidar_project{
//构造函数，构建求解器，确定求解器类型
G2oGraphOptimizer::G2oGraphOptimizer(const std::string& solver_type){
    graph_ptr_.reset(new g2o::SparseOptimizer());
    /**
     * \brief create solvers based on their short name
     *
     * Factory to allocate solvers based on their short name.
     * The Factory is implemented as a sigleton and the single
     * instance can be accessed via the instance() function.
     */
    //创建一个优化器，线建立一个优化器生成器，然后按照指定的优化器名生成对应的solver,可以指定lm_var,gn_var,dl_var等等多种类型。
    g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    //构建求解器
    g2o::OptimizationAlgorithm* solver = solver_factory->construct(solver_type,solver_property);
    graph_ptr_->setAlgorithm(solver);

    if(!graph_ptr_->solver()){
        LOG(ERROR)<<"G2O 优化器创建失败";
    }
    robust_kernel_factory_ = g2o::RobustKernelFactory::instance();

}


bool G2oGraphOptimizer::Optimize(){
    static int optimize_count = 0;
    if(graph_ptr_->edges().size() < 1){
        return false;
    }

    TicToc optimize_time;
    //稀疏优化器指针
    //初始化，1.把所有的顶点（Vertex）插入到vset的集合中
    // 2. 遍历vset集合，取出每个顶点的边（这里每个边都有一个level的概念，默认情况下，g2o只处理level=0的边，=1的误差大，放弃处理
    // 3. 对上述的_activeEdges和_activeVertices按照ID号进行排序，其中Vertex的ID号是自己设置的，而Edge的ID号是g2o内部有个变量进行赋值的
    // 4. 对于上述的_activeVertices，剔除掉固定点（fixed）之后，把所有的顶点按照**不被**margin在前，被margin在后的顺序排成vector类型，
    // 变量为_ivMap，这个变量很重要，基本上后面的所有程序都是用这个变量进行遍历的
    graph_ptr_->initializeOptimization();
    // 通过图的边，从一个设定为origin的顶点开始计算各个其他顶点的直。
    graph_ptr_->computeInitialGuess();
    // 计算激活节点的总误差
    graph_ptr_->computeActiveErrors();
    // 关闭调试输出
    graph_ptr_->setVerbose(false);

    double chi2 = graph_ptr_->chi2();//chi2是最小二乘问题中该边的代价，
    int iterations = graph_ptr_->optimize(max_iteration_num_);//开始优化
    LOG(INFO) << std::endl <<"----- 完成第 "<<"次后端优化 ----"<<std::endl
              << "顶点数: "<<graph_ptr_->vertices().size() << ", 边数"<<graph_ptr_->edges().size() << std::endl
              <<"迭代次数: "<<iterations <<" / "<<max_iteration_num_<<std::endl
              <<"耗时： "<<optimize_time.toc()<<std::endl
              <<"优化后误差变化："<<chi2 <<"---> "<< graph_ptr_->chi2()
              << std::endl <<std::endl;
`
    return true;

}

//获取优化位姿
bool G2oGraphOptimizer::GetOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose){
    optimized_pose.clear();
    int vertex_num = graph_ptr_->vertices().size();

    for(int i=0; i<vertex_num; i++){
        g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(i));
        Eigen::Isometry3d pose = v->estimate();
        optimized_pose.push_back(pose.matrix().cast<float>());
    }
    return true;
}

//获取节点数量
int G2oGraphOptimizer::GetNodeNum(){
    return graph_ptr_->vertices().size();
}

void G2oGraphOptimizer::AddSe3Node(const Eigen::Isometry3d& pose,bool need_fix){
    g2o::VertexSE3* vertex(new g2o::VertexSE3());
    vertex->setId(graph_ptr_->vertices().size());//添加节点索引和位姿，一直往后加
    vertex->setEstimate(pose);
    if(need_fix){
        vertex->setFixed(true);//是否固定
    }
    graph_ptr_->addVertex(vertex);
}


//鲁棒核函数选择
void G2oGraphOptimizer::SetEdgeRobustKernel(std::string robust_kernel_name,double robust_kernel_size){
    robust_kernel_name_ = robust_kernel_name;
    robust_kernel_size_ = robust_kernel_size;
    need_robust_kernel_ = true;
}


//添加SE3边
void G2oGraphOptimizer::AddSe3Edge(int vertex_index1,
                                    int vertex_index2,
                                    const Eigen::Isometry3d& relative_pose,//帧间相对位姿
                                    const Eigen::VectorXd noise){//信息矩阵，权重
    Eigen::MatrixXd infromation_matrix = CalculateSe3EdgeInformationMatrix(noise);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index1));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(vertex_index2));
    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);//观测
    edge->setInformation(infromation_matrix);
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    graph_ptr_->addEdge(edge);
    if(need_robust_kernel_){
        AddRobustKernel(edge,robust_kernel_name_,robust_kernel_size_);
    }

}

//信息矩阵计算
Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3EdgeInformationMatrix(Eigen::VectorXd noise){
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6,6);//6x6的矩阵为什么
    information_matrix = CalculateDiagMatrix(noise);
    return information_matrix;
}

void G2oGraphOptimizer::AddRobustKernel(g2o::OptimizableGraph::Edge* edge,const std::string& kernel_type, double kernel_size){
    if(kernel_type == "NONE"){
        return;
    }

    g2o::RobustKernel* kernel = robust_kernel_factory_->construct(kernel_type);
    if(kernel == nullptr){
        std::cerr <<"Warning : invalid robust kernel type: "<<kernel_type<<std::endl;
        return;
    }
    kernel->setDelta(kernel_size);
    edge->setRobustKernel(kernel);

}


Eigen::MatrixXd G2oGraphOptimizer::CalculateDiagMatrix(Eigen::VectorXd noise){
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(),noise.rows());
    for(int i=0; i<noise.rows(); i++){
        information_matrix(i,i) /= noise(i);
    }
    return information_matrix;
}

//添加先验边，相对于原点的位姿
void G2oGraphOptimizer::AddSe3PriorXYZEdge(int se3_vertex_index,
                                           const Eigen::Vector3d& xyz,
                                           Eigen::VectorXd noise){
    Eigen::MatrixXd information_matrix = CalculateDiagMatrix(noise);
    g2o::VertexSE3* v_se3 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);                
    
}

//添加姿态先验边
void G2oGraphOptimizer::AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                       const Eigen::Quaterniond& quat,//姿态作为先验边
                                       Eigen::VectorXd noise){
    Eigen::MatrixXd information_matrix = CalculateSe3PriorQuaternionEdgeInformationMatrix(noise);
    g2o::VertexSE3* v_se3 = dynamic_cast<g2o::VertexSE3*>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph_ptr_->addEdge(edge);

}

//姿态观测的信息矩阵
Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3PriorQuaternionEdgeInformationMatrix(Eigen::VectorXd noise){
    Eigen::MatrixXd information_matrix;
    return information_matrix;
}



}