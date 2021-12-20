#ifndef POINTCLOUDSEG_H
#define POINTCLOUDSEG_H
#include "utils/typeConvert.h"
#include "utils/common.h"
#include "groundSeg/ground_segmentation.h"
#include "utils/channel.h"

class PointCloudSeg {
private:
    // 用于特征点提取点云的一些基本信息
    SegInfo segMsg_;
    
    pcl::PointCloud<PointT_I>::Ptr laserCloudIn_;

    pcl::PointCloud<PointT_I>::Ptr orderedFullCloud_;
    pcl::PointCloud<PointT_I>::Ptr fullInfoCloud_;

    pcl::PointCloud<PointT_I>::Ptr groundCloud_;// 地面点云
    pcl::PointCloud<PointT_I>::Ptr segmentedCloud_;// 分割出的非地面点云
    pcl::PointCloud<PointT>::Ptr  cloudforGroundSeg_;

    PointT_I nanPoint; //定义一个无效点

    cv::Mat rangeMat_;// 保存点的距离
    cv::Mat labelMat_;// 标记点的类型
    cv::Mat groundMat_;// 标记地面点, 1是地面点
    int labelCount_;
    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;
    int *allPushedIndX;
    int *allPushedIndY;
    int *queueIndX;
    int *queueIndY;

    /********* 自研方法的参数 ********/
    // 基于线分割的地面分割算法对象
    GroundSegmentation groundSegTool_;

    int indexMat_[N_SCAN][Horizon_SCAN] = {{-1}}; // 索引矩阵
    double depthMat_[N_SCAN][Horizon_SCAN] = {{0}};// 点云深度矩阵
    bool verticalFlagMat_[N_SCAN][Horizon_SCAN];   // 标记点(i,j)是否属于竖直元素
    int groundSegMat_[N_SCAN][Horizon_SCAN] = {{0}}; // 地面标记矩阵
public:
    std::vector<int> groundLabel_;// 程序输出信息，1是地面点。
private:
    // 类成员变量初始化
    void variablesInit();
    // 重置变量
    void resetVariables();
    // 获取雷达点云起始结束的水平角度(弧度值)
    void findStartEndHorizontalDeg();
    // 点云数据结构化
    void unorderedCloud2Structure(const PointCloud_I::Ptr& laserCloudIn);
    // 地面提取方法
    void groundRemoval();
    // 对非对面点云进行聚类和分割，剔除未形成聚类的点
    void cloudSegmentation();
    // labelMat的BFS遍历进行标记聚类(连通图算法)
    void labelComponents(int row, int col);
public:
    PointCloudSeg(const InitParams& params);
    // 重投影及分割聚类
    void cloudHandler(const PointCloud_I::Ptr& laserCloudIn);
    // 发布预分割的点云，以及分割信息
    void publishSegInfo(PointCloud_I::Ptr& laserCloudOut, SegInfo& segInfoOut);
};

#endif