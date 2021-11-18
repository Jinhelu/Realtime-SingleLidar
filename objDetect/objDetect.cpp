#include "objDetect.h"

// 引导目标检测构造函数
ObjDetect::ObjDetect(const InitParams& params){
    params_.intensity_threshold = params.intensity_threshold;
    params_.n_reflectPoint_closest = params.n_reflectPoint_closest;
    params_.r_reflectPoint_search = params.r_reflectPoint_search;
    params_.std_target_size = params.std_target_size;
    params_.voxel_leaf_size = params.voxel_leaf_size;
}

// 获取引导目标中心点云
void ObjDetect::getObjectCloud(const PointCloud_I::Ptr& inputCloud, PointCloud_I::Ptr& targetCenterCloud,
                        PointCloud_I::Ptr& targetCubeCloud){
    PointCloud_I::Ptr intensityFiltedCloud(new PointCloud_I);
    //滤波找高反射率点(一次滤波)
    for(int i=0; i<inputCloud->size(); i++){
        if(inputCloud->points[i].intensity > params_.intensity_threshold){
            intensityFiltedCloud->points.push_back(inputCloud->points[i]);
        }
    }
    //二次滤波，用kd树去除散点
    if(!intensityFiltedCloud->empty()){
        targetCenterCloud -> clear();
        //近邻搜索滤除误判点
        float SearchRadius = params_.r_reflectPoint_search;//搜索半径
        pcl::KdTreeFLANN<PointT_I> kdtree; //建立kd树求近邻
        kdtree.setInputCloud(intensityFiltedCloud);//设置输入点云

        for(int i=0; i<intensityFiltedCloud->size(); i++) {
            vector<int> pointIdxRadiusSearch;//距离搜索结果ID向量
            vector<float> pointRadiusSquaredDistance;//距离搜索结果距离向量
            PointT_I searchPoint(intensityFiltedCloud->points[i]);//核心点设置
            //指定半径的近邻搜索   中心点        半径           结果ID向量  结果距离向量
            kdtree.radiusSearch(searchPoint, SearchRadius,
                                pointIdxRadiusSearch, pointRadiusSquaredDistance);

            //没有足够的近邻点认为是误判,不加入二次目标当中
            if(pointIdxRadiusSearch.size()>params_.n_reflectPoint_closest){
                targetCenterCloud->points.push_back(intensityFiltedCloud->points[i]);
            }
        }
    }
}

// 获取引导目标整体点云
bool ObjDetect::getObjectCloudPosition(const PointCloud_I::Ptr &targetCenterCloud, GuideObject& guideObject, double& time_stt_us){
    // 目标点云太少，直接返回错误
    if(targetCenterCloud->size() < 5) return false;
    
    float maxDistance = 0;// 点云簇中两点间的最大距离
    float secDistance = 0;// 点云簇中两点间的第二大距离
    PointT heartPoint(0,0,0);//中心点坐标

    for(int i=0; i<targetCenterCloud->size(); i++){
        heartPoint.x += targetCenterCloud->points[i].x;
        heartPoint.y += targetCenterCloud->points[i].y;
        heartPoint.z += targetCenterCloud->points[i].z;
        for(int j=i+1; j<targetCenterCloud->size(); j++){
            float deltaX = targetCenterCloud->points[j].x - targetCenterCloud->points[i].x;
            float deltaY = targetCenterCloud->points[j].y - targetCenterCloud->points[i].y;
            float deltaZ = targetCenterCloud->points[j].z - targetCenterCloud->points[i].z;
            float distance = sqrtf(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);
            if(distance > maxDistance) {
                secDistance = maxDistance;
                maxDistance = distance;
            }else if(distance > secDistance){
                secDistance = distance;
            }
        }
    }
    // 目标点云簇中心点的坐标,
    heartPoint.x /= targetCenterCloud->size();
    heartPoint.y /= targetCenterCloud->size();
    heartPoint.z /= targetCenterCloud->size();
    //求实际测得的对角线的长度（引导目标是矩形）
    double target_delta;
    if(maxDistance-secDistance < 0.1 || secDistance == 0) target_delta = maxDistance;
    else target_delta = secDistance;
    float target_delta_threshold = params_.std_target_size - params_.voxel_leaf_size;//理论的精确的对角线长度
    float allow_deviation = 3*params_.voxel_leaf_size; //上述两个长度的可允许偏差
    double deviation = target_delta - target_delta_threshold ; //减去计算实际对角线的偏大误差

    //如果误差在合理范围内,就计算出目标点的坐标 对角线长度 目标对象的距离
    if(fabs(deviation) < allow_deviation) {
        guideObject.timeStamp_us = time_stt_us;
        guideObject.position.x = heartPoint.x;
        guideObject.position.y = heartPoint.y;
        guideObject.position.z = heartPoint.z;

        double target_distance = guideObject.getTargetDist();
        double horizontal_angel = guideObject.getHorizontalDeg();
        double vertical_angel = guideObject.getVerticalDeg();
        cout << "  水平角: " << (horizontal_angel>0?"左":"右") << fabs(horizontal_angel) << "°"
             <<"  垂直角: " << (vertical_angel>0?"上":"下") << fabs(vertical_angel) << "°" << endl;
        cout << "目标对象距离: " << target_distance << endl << "*****************************" << endl << endl;
        return true;
    }else {
        cout << "目标点云非跟随对象" << endl;
        return false;
    }
}

// getObjectCloudVel获取引导目标整体点云速度
bool ObjDetect::getObjectCloudVel(GuideObject& guideObject, GuideObject& lastGuideObj){
    double delta_time_s = (guideObject.timeStamp_us - lastGuideObj.timeStamp_us)/(1000*1000);
    //目标与雷达的相对速度计算　拟合背部平面求出法向量作为人员朝向
    if(lastGuideObj.position.x != 0 && delta_time_s != 0){
        double Vx = (guideObject.position.x - lastGuideObj.position.x)/delta_time_s;
        double Vy = (guideObject.position.y - lastGuideObj.position.y)/delta_time_s;
        guideObject.vel.setVelocity(Vx, Vy);//此处已经计算过合速度
        /*
        cout << "delta_time_s: " << delta_time_s << " velo_x:" << guideObject.vel.v_x
            <<" velo_y:" <<  guideObject.vel.v_y << " velo_combine:" <<  guideObject.vel.v_combine << endl;
        */
        return true;
    } else {
        cout << "Velocity Analysis failed!" << endl;
        return false;
    }
}

//  getObjectCubeCloud滤除引导目标点云，生成非地面点非引导人员点云簇
void ObjDetect::delObjectCubeCloud(GuideObject& guideObject, PointCloud_I::Ptr& not_ground_cloud, PointCloud_I::Ptr& map_build_cloud){
    //人体模型，长宽1m，高2m
    float Guid_Xmin = guideObject.position.x - 0.5f;
    float Guid_Xmax = guideObject.position.x + 0.5f;
    float Guid_Ymin = guideObject.position.y - 0.5f;
    float Guid_Ymax = guideObject.position.y + 0.5f;
    float Guid_Zmin = guideObject.position.z - 1.4f;
    float Guid_Zmax = guideObject.position.z + 0.6f;

    map_build_cloud->clear();
    for(int i=0; i<not_ground_cloud->size(); i++){
        bool removeFlag =
                not_ground_cloud->points[i].x>Guid_Xmin && not_ground_cloud->points[i].x<Guid_Xmax &&
                not_ground_cloud->points[i].y>Guid_Ymin && not_ground_cloud->points[i].y<Guid_Ymax &&
                not_ground_cloud->points[i].z>Guid_Zmin && not_ground_cloud->points[i].z<Guid_Zmax;
        // 如果不是引导目标，则添加进生成地图的点云中
        if(!removeFlag){
            map_build_cloud->points.push_back(not_ground_cloud->points[i]);
        }
    }
}