/***
 * Des:GNSS坐标作为观测时使用的先验边
 * 0927
 * ***/
#ifndef EDGE_SE3_PRIORIXYZ_HPP_
#define EDGE_SE3_PRIORIXYZ_HPP_


#include <g2o/types/slam3d/types_slam3d.h> //点
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>//线面

namespace g2o{
//本质是添加观测和计算误差
class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3,Eigen::Vector3d,g2o::VertexSE3>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeSE3PriorXYZ():g2o::BaseUnaryEdge<3,Eigen::Vector3d,g2o::VertexSE3>(){}

        //计算误差,别写成computer。。。。
        void computeError() override{
            const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
            Eigen::Vector3d estimate = v1->estimate().translation();
            _error = estimate - _measurement;//measurement采用模板类
        }

        void setMeasurement(const Eigen::Vector3d& m) override{
            _measurement = m;
        }

        //设定观测量和信息矩阵
        virtual bool read(std::istream& is) override{
            Eigen::Vector3d v;
            is >> v(0) >> v(1) >> v(2);
            setMeasurement(Eigen::Vector3d(v));
            for(int i=0; i<information().rows(); i++){
                for(int j=i;j<information().cols(); j++){
                    is>>information()(i,j);
                    if(i!=j){
                        information()(j,i) = information()(i,j);//对称
                    }
                }
            }
            return true;

        }
        //不允许后续其他类覆盖write, 若将成员成员函数声明为const，则该函数不允许修改类的数据成员。
        //作为一种良好的编程风格，在声明一个成员函数时，若该成员函数并不对数据成员进行修改操作，应尽可能将该成员函数声明为const 成员函数。
        virtual bool write(std::ostream& os) const override{
            Eigen::Vector3d v = _measurement;
            os<<v(0)<<" "<<v(1)<<" "<<v(2)<<" ";
            for(int i=0; i< information().rows(); i++){
                for(int j=i; j<information().cols(); j++){
                    os<<" "<<information()(i,j);
                }
            }
            return os.good();
        }

        
        

};
}



#endif