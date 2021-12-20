#ifndef LIDARODOMETRY_H
#define LIDARODOMETRY_H

#include "utils/channel.h"
#include "utils/common.h"
#include "utils/typeConvert.h"

#define DISTORTION 0

class LidarOdometry
{
private:
    Channel<ExtractionOut>& input_channel_;
    Channel<OdometryOut>& output_channel_;
    std::thread _run_thread;// 里程计计算线程

    // 当前帧特征点参数
    pcl::PointCloud<PointT_I>::Ptr cornerPointsSharp_;
    pcl::PointCloud<PointT_I>::Ptr cornerPointsLessSharp_;
    pcl::PointCloud<PointT_I>::Ptr surfPointsFlat_;
    pcl::PointCloud<PointT_I>::Ptr surfPointsLessFlat_;

    // 上一帧特征点参数
    pcl::PointCloud<PointT_I>::Ptr laserCloudCornerLast_;
    pcl::PointCloud<PointT_I>::Ptr laserCloudSurfLast_;
    int laserCloudCornerLastNum_;
    int laserCloudSurfLastNum_;

    pcl::PointCloud<PointT_I>::Ptr laserCloudOri_;
    // coeffSel_中点云的[x,y,z]是整个平面的单位法量， intensity是平面外一点到该平面的距离，
    pcl::PointCloud<PointT_I>::Ptr coeffSel_;
    // kd-tree搜索使用的参数
    pcl::KdTreeFLANN<PointT_I>::Ptr kdtreeCornerLast_;
    pcl::KdTreeFLANN<PointT_I>::Ptr kdtreeSurfLast_;
    std::vector<int> pointSearchInd_;
    std::vector<float> pointSearchSqDis_;

    // 特征点匹配的参数
    PointT_I pointOri_, pointSel_, tripod1_, tripod2_, tripod3_, pointProj_, coeff_;
    int pointSelCornerInd_[N_SCAN*Horizon_SCAN];// 选择的特征角点id
    float pointSearchCornerInd1_[N_SCAN*Horizon_SCAN];// 最近邻搜索得到的另外两个角点的id
    float pointSearchCornerInd2_[N_SCAN*Horizon_SCAN];

    int pointSelSurfInd_[N_SCAN*Horizon_SCAN];// 选择的平面点id
    float pointSearchSurfInd1_[N_SCAN*Horizon_SCAN];// 最近邻搜索得到的另外3个平面点的id
    float pointSearchSurfInd2_[N_SCAN*Horizon_SCAN];
    float pointSearchSurfInd3_[N_SCAN*Horizon_SCAN];

    bool systemInited; // 系统初始化完成
    // 里程计相关参数
    float transformCur[6];// 当前帧相对上一帧的位姿变换
    float transformSum[6];// 当前帧相对起始时刻的位姿变换

    bool isDegenerate;
    cv::Mat matP;

private:
    void initializationValue();// 各种参数的初始化
    // 使用imu数据更新预测的当前帧初始位姿信息
    void updateInitialPoseByImu();
    // 矫正雷达点云,将点云向初始时刻对齐(单帧内)
    void transformCloudToStart(PointT_I const *const pi, PointT_I *const po);
    // 将点云转换到下一帧的开始
    void transformCloudToEnd(PointT_I const *const pi, PointT_I *const po);
    // 获取相邻的平面点特征
    void findCorrespondingCornerFeatures(int iterCount);
    // 获取相邻的角点特征
    void findCorrespondingSurfFeatures(int iterCount);
    // 使用平面点特征计算变换矩阵
    bool calculateTransformationSurf(int iterCount);
    // 使用角点特征计算变换矩阵
    bool calculateTransformationCorner(int iterCount);
    //更新里程计信息
    void updateTransformation();
    // 累加里程计的旋转量
    void accumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                            float &ox, float &oy, float &oz);
    // 位姿矩阵的累计变化量，相对于第一帧的旋转矩阵
    void integrateTransformation();
    // 进行系统初始化
    void checkSystemInitialization();
    // 发布里程计数据，将线程中计算得到的里程计传递出去
    void publishOdometry();
    // 更新上一帧点云信息
    void updateCloudsLast();
public:
    LidarOdometry(Channel<ExtractionOut>& input_channel,
                     Channel<OdometryOut>& output_channel);
    ~LidarOdometry();
    // 执行里程计计算程序
    void runLidarOdometry();
};

#endif