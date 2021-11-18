#ifndef OBJDETECT_H
#define OBJDETECT_H
#include <utils/common.h>

// Velocity 定义引导人员运动速度信息
struct Velocity {
    double v_x;        // x方向速度
    double v_y;        // y方向速度
    double v_combine;  // xy方向合成速度
    Velocity(){
        setVelocity(0,0);
    }
    void setVelocity(double Velox,double Veloy){
      v_x = Velox;
      v_y = Veloy;
      v_combine = sqrt(pow(Velox,2) + pow(Veloy,2));
    }
};

// GuideObject 定义引导人员位置速度信息
struct GuideObject{
    PointT position;
    Velocity vel;
    double timeStamp_us;

    GuideObject() {
        position.x = 0;position.y = 0;position.z = 0;
        vel.setVelocity(0, 0);
        timeStamp_us = 0;
    }
    GuideObject(float posiX, float posiY, float posiZ) {
        position.x = posiX;
        position.y = posiY;
        position.z = posiZ;
        vel.setVelocity(0, 0);
        timeStamp_us = 0;
    }
    double getTargetDist(){
        return sqrt(pow(position.x, 2) + pow(position.y, 2) + pow(position.z, 2));
    }
    double getHorizontalDeg() {
        return 180.0 * (atan(position.y / position.x)) / 3.1415926f;
    }
    double getVerticalDeg() {
        return 180.0 * (atan(position.z / position.x)) / 3.1415926f;
    }
};

struct ObjDetectParams {
    int n_reflectPoint_closest;         // 高反射率点的近邻点数量
    float voxel_leaf_size;         // 体素滤波的网格大小值
    float intensity_threshold;     // 雷达反射强度阈值
    float r_reflectPoint_search;          // 高反射率点的近邻搜索范围
    float std_target_size;               // 对角线长度

    ObjDetectParams() : 
        voxel_leaf_size(0.035),
        intensity_threshold(196),
        r_reflectPoint_search(0.1),
        n_reflectPoint_closest(4),
        std_target_size(0.35355) {}
};

class ObjDetect {
private:
    ObjDetectParams params_;

public:
    ObjDetect(const InitParams& params);
    
    // 获取引导目标中心点云
    void getObjectCloud(const PointCloud_I::Ptr& inputCloud, PointCloud_I::Ptr& targetCenterCloud,
                        PointCloud_I::Ptr& targetCubeCloud);
    // 获取引导目标整体点云位置
    bool getObjectCloudPosition(const PointCloud_I::Ptr &targetCenterCloud, GuideObject& guideObject, double& time_stt_us);
    // 获取引导目标整体点云速度
    bool getObjectCloudVel(GuideObject& guideObject, GuideObject& lastGuideObj);
    // 滤除引导目标点云，生成非地面点非引导人员点云簇
    void delObjectCubeCloud(GuideObject& guideObject, PointCloud_I::Ptr& not_ground_cloud, PointCloud_I::Ptr& map_build_cloud);
};


#endif