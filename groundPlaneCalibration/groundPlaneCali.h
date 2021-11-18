#ifndef GROUNDPLANECALI_H
#define GROUNDPLANECALI_H

#include <pcl/ModelCoefficients.h>        // 模型系数定义头文件
#include <pcl/filters/project_inliers.h>  // 投影滤波类头文件
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "utils/common.h"

struct GroundCaliParams{
    float max_deviation_deg;     //有精度地面在雷达坐标系中的最大偏离pitch、roll轴角度
    float plane_dist_threshold;  //地面方程参数估计时的距离阈值
    float groundCali_deg_threshold; //估计地面偏离的角度大于阈值时，进行地面矫正
    int n_ground_estimate;       //进行地面方程参数估计的迭代次数

    GroundCaliParams():
        max_deviation_deg(15),
        plane_dist_threshold(0.05),
        groundCali_deg_threshold(2),
        n_ground_estimate(40){}
};

class GroundPlaneCali{
private:
    GroundCaliParams params_; 
public:
    GroundPlaneCali(const InitParams& params);

    // groundEquationEstimate 获取地面估计的方程 ax+by+cz+d=0
    void groundEquationEstimate(PointCloud::Ptr& pointcloud, pcl::ModelCoefficients& PlaneCoeff2Show, bool& needGroundCali);

    // getCaliRotateMatrix 对估计地面进行地平面校准，获取旋转矩阵
    Eigen::Matrix3f getCaliRotateMatrix(pcl::ModelCoefficients& PlaneCoeff2Show);

private:
    // 使用ransac方法获取平面方程数据
    void planeMatchRANSAC(PointCloud::Ptr& pointcloud, std::vector<int>& inliers, pcl::ModelCoefficients& PlaneCoeff2Show);

    // CreateRotateMatrix 计算地面估计向量与地面标准法向量(0,0,1)间的旋转矩阵
    Eigen::Matrix3f CreateRotateMatrix(Eigen::Vector3f& groundEstimateNormal);
};

#endif