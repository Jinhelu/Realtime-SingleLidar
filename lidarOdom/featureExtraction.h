#ifndef FEATUREEXA_H
#define FEATUREEXA_H
#include "utils/common.h"
#include "utils/channel.h"

class FeatureExtraction{
private:
    // 预处理后分割点云的一些信息
    SegInfo segMsg_;
    pcl::PointCloud<PointT_I>::Ptr segmentedCloud_;
    // 输出信息
    Channel<ExtractionOut>& output_channel_; 
    

    pcl::PointCloud<PointT_I>::Ptr cornerPointsSharp_;
    pcl::PointCloud<PointT_I>::Ptr cornerPointsLessSharp_;
    pcl::PointCloud<PointT_I>::Ptr surfPointsFlat_;
    pcl::PointCloud<PointT_I>::Ptr surfPointsLessFlat_;

    pcl::PointCloud<PointT_I>::Ptr surfPointsLessFlatScan_;
    pcl::PointCloud<PointT_I>::Ptr surfPointsLessFlatScanDS_;

    pcl::VoxelGrid<PointT_I> downSizeFilter_;

    std::vector<Smoothness> cloudSmoothness_;
    float cloudCurvature_[N_SCAN*Horizon_SCAN] = {-1};
    int cloudNeighborPicked_[N_SCAN*Horizon_SCAN] = {-1};

    // surfPointsFlat标记为-1，surfPointsLessFlat为不大于0的标签, cornerPointsSharp标记为2，cornerPointsLessSharp标记为1
    int cloudLabel_[N_SCAN*Horizon_SCAN] = {0};

private:
    void initializationValue();
    // 雷达坐标系到运动坐标系的变换
    void coordinateTransform();
    // 计算平滑程度
    void calculateSmoothness();
    // 标记点云中相互遮挡又靠的很近的点
    void markOccludePoints();
    // 提取四种特征点
    void extractFeatures();
public:
    FeatureExtraction(Channel<ExtractionOut>& output_channel);
    // 发布四种特征点云信息
    void publishFeatureCloud();
    
    void runFeatureAssociation(const pcl::PointCloud<PointT_I>::Ptr segmentedCloud,  SegInfo& segInfo);
};


#endif