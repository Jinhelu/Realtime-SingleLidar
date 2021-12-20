#include "lidarOdometry.h"

LidarOdometry::LidarOdometry(Channel<ExtractionOut>& input_channel,
                     Channel<OdometryOut>& output_channel) : 
                     input_channel_(input_channel),
                     output_channel_(output_channel) {
    // 参数初始化
    initializationValue();  
    _run_thread = std::thread (&LidarOdometry::runLidarOdometry, this);
}
LidarOdometry::~LidarOdometry(){
    _run_thread.join();
}
// 各种参数的初始化
void LidarOdometry::initializationValue(){
    cornerPointsSharp_.reset(new pcl::PointCloud<PointT_I>());
    cornerPointsLessSharp_.reset(new pcl::PointCloud<PointT_I>());
    surfPointsFlat_.reset(new pcl::PointCloud<PointT_I>());
    surfPointsLessFlat_.reset(new pcl::PointCloud<PointT_I>());

    laserCloudCornerLast_.reset(new pcl::PointCloud<PointT_I>());
    laserCloudSurfLast_.reset(new pcl::PointCloud<PointT_I>());
    laserCloudOri_.reset(new pcl::PointCloud<PointT_I>());
    coeffSel_.reset(new pcl::PointCloud<PointT_I>());
    
    kdtreeCornerLast_.reset(new pcl::KdTreeFLANN<PointT_I>());
    kdtreeSurfLast_.reset(new pcl::KdTreeFLANN<PointT_I>());
    
    systemInited = false;
    isDegenerate = false;
    for(int i = 0; i < 6; ++i){
        transformCur[i] = 0;
        transformSum[i] = 0;
    }
    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

}

// 矫正雷达点云,将点云向初始时刻对齐(单帧内)
void LidarOdometry::transformCloudToStart(PointT_I const *const pi, PointT_I *const po){
    double s; // 插值系数
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity))*10000 / Horizon_SCAN;
        //s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    po->x = cos(ry) * x2 - sin(ry) * z2;
    po->y = y2;
    po->z = sin(ry) * x2 + cos(ry) * z2;
    po->intensity = pi->intensity;
}

// 将点云转换到下一帧的开始
void LidarOdometry::transformCloudToEnd(PointT_I const *const pi, PointT_I *const po){
    double s; // 插值系数
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity))*10000 / Horizon_SCAN;
        //s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
    float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
    float z1 = (pi->z - tz);

    float x2 = x1;
    float y2 = cos(rx) * y1 + sin(rx) * z1;
    float z2 = -sin(rx) * y1 + cos(rx) * z1;

    float x3 = cos(ry) * x2 - sin(ry) * z2;
    float y3 = y2;
    float z3 = sin(ry) * x2 + cos(ry) * z2;

    rx = transformCur[0];
    ry = transformCur[1];
    rz = transformCur[2];
    tx = transformCur[3];
    ty = transformCur[4];
    tz = transformCur[5];

    float x4 = cos(ry) * x3 + sin(ry) * z3;
    float y4 = y3;
    float z4 = -sin(ry) * x3 + cos(ry) * z3;

    float x5 = x4;
    float y5 = cos(rx) * y4 - sin(rx) * z4;
    float z5 = sin(rx) * y4 + cos(rx) * z4;

    float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
    float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
    float z6 = z5 + tz;

    po->x = x6;
    po->y = y6;
    po->z = z6;
    po->intensity = int(pi->intensity);
}

// 使用imu数据更新预测值
void LidarOdometry::updateInitialPoseByImu(){

}

// 获取相邻的角点特征
void LidarOdometry::findCorrespondingCornerFeatures(int iterCount){
    int cornerPointsSharpNum = cornerPointsSharp_->points.size();

    for (int i = 0; i < cornerPointsSharpNum; i++) {

        transformCloudToStart(&cornerPointsSharp_->points[i], &pointSel_);

        if (iterCount % 5 == 0) {

            kdtreeCornerLast_->nearestKSearch(pointSel_, 1, pointSearchInd_, pointSearchSqDis_);
            int closestPointInd = -1, minPointInd2 = -1;
            
            if (pointSearchSqDis_[0] < nearestFeatureSearchSqDist) {
                closestPointInd = pointSearchInd_[0];
                int closestPointScan = int(laserCloudCornerLast_->points[closestPointInd].intensity);

                float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist;
                for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
                    if (int(laserCloudCornerLast_->points[j].intensity) > closestPointScan + 2.5) {
                        break;
                    }

                    pointSqDis = (laserCloudCornerLast_->points[j].x - pointSel_.x) * 
                                    (laserCloudCornerLast_->points[j].x - pointSel_.x) + 
                                    (laserCloudCornerLast_->points[j].y - pointSel_.y) * 
                                    (laserCloudCornerLast_->points[j].y - pointSel_.y) + 
                                    (laserCloudCornerLast_->points[j].z - pointSel_.z) * 
                                    (laserCloudCornerLast_->points[j].z - pointSel_.z);

                    if (int(laserCloudCornerLast_->points[j].intensity) > closestPointScan) {
                        if (pointSqDis < minPointSqDis2) {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    }
                }
                for (int j = closestPointInd - 1; j >= 0; j--) {
                    if (int(laserCloudCornerLast_->points[j].intensity) < closestPointScan - 2.5) {
                        break;
                    }

                    pointSqDis = (laserCloudCornerLast_->points[j].x - pointSel_.x) * 
                                    (laserCloudCornerLast_->points[j].x - pointSel_.x) + 
                                    (laserCloudCornerLast_->points[j].y - pointSel_.y) * 
                                    (laserCloudCornerLast_->points[j].y - pointSel_.y) + 
                                    (laserCloudCornerLast_->points[j].z - pointSel_.z) * 
                                    (laserCloudCornerLast_->points[j].z - pointSel_.z);

                    if (int(laserCloudCornerLast_->points[j].intensity) < closestPointScan) {
                        if (pointSqDis < minPointSqDis2) {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    }
                }
            }

            pointSearchCornerInd1_[i] = closestPointInd;
            pointSearchCornerInd2_[i] = minPointInd2;
        }

        if (pointSearchCornerInd2_[i] >= 0) {

            tripod1_ = laserCloudCornerLast_->points[pointSearchCornerInd1_[i]];
            tripod2_ = laserCloudCornerLast_->points[pointSearchCornerInd2_[i]];

            float x0 = pointSel_.x;
            float y0 = pointSel_.y;
            float z0 = pointSel_.z;
            float x1 = tripod1_.x;
            float y1 = tripod1_.y;
            float z1 = tripod1_.z;
            float x2 = tripod2_.x;
            float y2 = tripod2_.y;
            float z2 = tripod2_.z;

            float m11 = ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1));
            float m22 = ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1));
            float m33 = ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1));

            float a012 = sqrt(m11 * m11  + m22 * m22 + m33 * m33);

            float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

            float la =  ((y1 - y2)*m11 + (z1 - z2)*m22) / a012 / l12;

            float lb = -((x1 - x2)*m11 - (z1 - z2)*m33) / a012 / l12;

            float lc = -((x1 - x2)*m22 + (y1 - y2)*m33) / a012 / l12;

            float ld2 = a012 / l12;

            float s = 1;
            if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(ld2);
            }

            if (s > 0.1 && ld2 != 0) {
                // [x,y,z]是直线的单位向量
                // intensity是直线外一点到该直线的距离
                coeff_.x = s * la; 
                coeff_.y = s * lb;
                coeff_.z = s * lc;
                coeff_.intensity = s * ld2;
                laserCloudOri_->push_back(cornerPointsSharp_->points[i]);
                coeffSel_->push_back(coeff_);
            }
        }
    }
}

// 获取相邻的平面点特征
void LidarOdometry::findCorrespondingSurfFeatures(int iterCount){
    int surfPointsFlatNum = surfPointsFlat_->points.size();
    for (int i = 0; i < surfPointsFlatNum; i++) {
        // 坐标变换到开始时刻，参数0是输入，参数1是输出
        transformCloudToStart(&surfPointsFlat_->points[i], &pointSel_);

        if (iterCount % 5 == 0) {
            // k点最近邻搜索，这里k=1
            kdtreeSurfLast_->nearestKSearch(pointSel_, 1, pointSearchInd_, pointSearchSqDis_);
            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

            // sq:平方，距离的平方值
            // 如果nearestKSearch找到的1(k=1)个邻近点满足条件
            if (pointSearchSqDis_[0] < nearestFeatureSearchSqDist) {
                closestPointInd = pointSearchInd_[0];
                
                // thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
                // 获取最临近点的scan值
                int closestPointScan = int(laserCloudSurfLast_->points[closestPointInd].intensity);

                // 主要功能是找到2个scan之内的最近点，并将找到的最近点及其序号保存
                // 之前扫描的保存到minPointSqDis2，之后的保存到minPointSqDis2
                float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
                // 往后方的特征点找
                for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
                    if (int(laserCloudSurfLast_->points[j].intensity) > closestPointScan + 2.5) {
                        break;
                    }
                    pointSqDis = (laserCloudSurfLast_->points[j].x - pointSel_.x) * 
                                    (laserCloudSurfLast_->points[j].x - pointSel_.x) + 
                                    (laserCloudSurfLast_->points[j].y - pointSel_.y) * 
                                    (laserCloudSurfLast_->points[j].y - pointSel_.y) + 
                                    (laserCloudSurfLast_->points[j].z - pointSel_.z) * 
                                    (laserCloudSurfLast_->points[j].z - pointSel_.z);

                    if (int(laserCloudSurfLast_->points[j].intensity) <= closestPointScan) {
                        if (pointSqDis < minPointSqDis2) {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    } else {
                        if (pointSqDis < minPointSqDis3) {
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }
                }
                // 往前方的特征点找
                for (int j = closestPointInd - 1; j >= 0; j--) {
                    if (int(laserCloudSurfLast_->points[j].intensity) < closestPointScan - 2.5) {
                        break;
                    }
                    pointSqDis = (laserCloudSurfLast_->points[j].x - pointSel_.x) * 
                                    (laserCloudSurfLast_->points[j].x - pointSel_.x) + 
                                    (laserCloudSurfLast_->points[j].y - pointSel_.y) * 
                                    (laserCloudSurfLast_->points[j].y - pointSel_.y) + 
                                    (laserCloudSurfLast_->points[j].z - pointSel_.z) * 
                                    (laserCloudSurfLast_->points[j].z - pointSel_.z);

                    if (int(laserCloudSurfLast_->points[j].intensity) >= closestPointScan) {
                        if (pointSqDis < minPointSqDis2) {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    } else {
                        if (pointSqDis < minPointSqDis3) {
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }
                }
            }
            pointSearchSurfInd1_[i] = closestPointInd;
            pointSearchSurfInd2_[i] = minPointInd2;
            pointSearchSurfInd3_[i] = minPointInd3;
        }

        // 前后都能找到对应的最近点在给定范围之内, 那么就开始计算距离
        // [pa,pb,pc]是tripod1_，tripod2_，tripod3_这3个点构成的一个平面的方向量，
        // ps是模长，它是三角形面积的2倍
        if (pointSearchSurfInd2_[i] >= 0 && pointSearchSurfInd3_[i] >= 0) {

            tripod1_ = laserCloudSurfLast_->points[pointSearchSurfInd1_[i]];
            tripod2_ = laserCloudSurfLast_->points[pointSearchSurfInd2_[i]];
            tripod3_ = laserCloudSurfLast_->points[pointSearchSurfInd3_[i]];

            float pa = (tripod2_.y - tripod1_.y) * (tripod3_.z - tripod1_.z) 
                        - (tripod3_.y - tripod1_.y) * (tripod2_.z - tripod1_.z);
            float pb = (tripod2_.z - tripod1_.z) * (tripod3_.x - tripod1_.x) 
                        - (tripod3_.z - tripod1_.z) * (tripod2_.x - tripod1_.x);
            float pc = (tripod2_.x - tripod1_.x) * (tripod3_.y - tripod1_.y) 
                        - (tripod3_.x - tripod1_.x) * (tripod2_.y - tripod1_.y);
            float pd = -(pa * tripod1_.x + pb * tripod1_.y + pc * tripod1_.z);

            float ps = sqrt(pa * pa + pb * pb + pc * pc);

            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            // 距离没有取绝对值， 两个向量的点乘，分母除以ps中已经除掉了，
            // 加pd原因:pointSel_与tripod1构成的线段需要相减
            float pd2 = pa * pointSel_.x + pb * pointSel_.y + pc * pointSel_.z + pd;

            float s = 1;
            if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel_.x * pointSel_.x
                        + pointSel_.y * pointSel_.y + pointSel_.z * pointSel_.z));
            }

            if (s > 0.1 && pd2 != 0) {
                // [x,y,z]是整个平面的单位法量
                // intensity是平面外一点到该平面的距离
                coeff_.x = s * pa;
                coeff_.y = s * pb;
                coeff_.z = s * pc;
                coeff_.intensity = s * pd2;
                // 未经变换的点放入laserCloudOri队列，距离，法向量值放入coeffSel
                laserCloudOri_->push_back(surfPointsFlat_->points[i]);
                coeffSel_->push_back(coeff_);
            }
        }
    }
}
// 使用平面点特征计算变换矩阵
bool LidarOdometry::calculateTransformationSurf(int iterCount){
    int pointSelNum = laserCloudOri_->points.size();
    cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    float srx = sin(transformCur[0]);
    float crx = cos(transformCur[0]);
    float sry = sin(transformCur[1]);
    float cry = cos(transformCur[1]);
    float srz = sin(transformCur[2]);
    float crz = cos(transformCur[2]);
    float tx = transformCur[3];
    float ty = transformCur[4];
    float tz = transformCur[5];

    float a1 = crx*sry*srz; float a2 = crx*crz*sry; float a3 = srx*sry; float a4 = tx*a1 - ty*a2 - tz*a3;
    float a5 = srx*srz; float a6 = crz*srx; float a7 = ty*a6 - tz*crx - tx*a5;
    float a8 = crx*cry*srz; float a9 = crx*cry*crz; float a10 = cry*srx; float a11 = tz*a10 + ty*a9 - tx*a8;

    float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz;
    float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry;

    float c1 = -b6; float c2 = b5; float c3 = tx*b6 - ty*b5; float c4 = -crx*crz; float c5 = crx*srz; float c6 = ty*c5 + tx*-c4;
    float c7 = b2; float c8 = -b1; float c9 = tx*-b2 - ty*-b1;

    // 构建雅可比矩阵，求解
    for (int i = 0; i < pointSelNum; i++) {

        pointOri_ = laserCloudOri_->points[i];
        coeff_ = coeffSel_->points[i];

        float arx = (-a1*pointOri_.x + a2*pointOri_.y + a3*pointOri_.z + a4) * coeff_.x
                    + (a5*pointOri_.x - a6*pointOri_.y + crx*pointOri_.z + a7) * coeff_.y
                    + (a8*pointOri_.x - a9*pointOri_.y - a10*pointOri_.z + a11) * coeff_.z;

        float arz = (c1*pointOri_.x + c2*pointOri_.y + c3) * coeff_.x
                    + (c4*pointOri_.x - c5*pointOri_.y + c6) * coeff_.y
                    + (c7*pointOri_.x + c8*pointOri_.y + c9) * coeff_.z;

        float aty = -b6 * coeff_.x + c4 * coeff_.y + b2 * coeff_.z;

        float d2 = coeff_.intensity;// 点到平面距离

        matA.at<float>(i, 0) = arx;
        matA.at<float>(i, 1) = arz;
        matA.at<float>(i, 2) = aty;
        matB.at<float>(i, 0) = -0.05 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {
        cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[3] = {10, 10, 10};
        for (int i = 2; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 3; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformCur[0] += matX.at<float>(0, 0);
    transformCur[2] += matX.at<float>(1, 0);
    transformCur[4] += matX.at<float>(2, 0);

    for(int i=0; i<6; i++){
        if(isnan(transformCur[i]))
            transformCur[i]=0;
    }

    float deltaR = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(rad2deg(matX.at<float>(1, 0)), 2));
    float deltaT = sqrt(pow(matX.at<float>(2, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1) {
        return false;
    }
    return true;
}
// 使用角点特征计算变换矩阵
bool LidarOdometry::calculateTransformationCorner(int iterCount){
    int pointSelNum = laserCloudOri_->points.size();

    cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    // 以下为开始计算A,A=[J的偏导],J的偏导的计算公式是什么?
    float srx = sin(transformCur[0]);
    float crx = cos(transformCur[0]);
    float sry = sin(transformCur[1]);
    float cry = cos(transformCur[1]);
    float srz = sin(transformCur[2]);
    float crz = cos(transformCur[2]);
    float tx = transformCur[3];
    float ty = transformCur[4];
    float tz = transformCur[5];

    float b1 = -crz*sry - cry*srx*srz; float b2 = cry*crz*srx - sry*srz; float b3 = crx*cry; float b4 = tx*-b1 + ty*-b2 + tz*b3;
    float b5 = cry*crz - srx*sry*srz; float b6 = cry*srz + crz*srx*sry; float b7 = crx*sry; float b8 = tz*b7 - ty*b6 - tx*b5;

    float c5 = crx*srz;

    for (int i = 0; i < pointSelNum; i++) {

        pointOri_ = laserCloudOri_->points[i];
        coeff_ = coeffSel_->points[i];

        float ary = (b1*pointOri_.x + b2*pointOri_.y - b3*pointOri_.z + b4) * coeff_.x
                    + (b5*pointOri_.x + b6*pointOri_.y - b7*pointOri_.z + b8) * coeff_.z;

        float atx = -b5 * coeff_.x + c5 * coeff_.y + b1 * coeff_.z;

        float atz = b7 * coeff_.x - srx * coeff_.y - b3 * coeff_.z;

        float d2 = coeff_.intensity;


        // A=[J的偏导]; B=[权重系数*(点到直线的距离)] 求解公式: AX=B
        // 为了让左边满秩，同乘At-> At*A*X = At*B
        matA.at<float>(i, 0) = ary;
        matA.at<float>(i, 1) = atx;
        matA.at<float>(i, 2) = atz;
        matB.at<float>(i, 0) = -0.05 * d2;
    }

    // transpose函数求得matA的转置matAt
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    // 通过QR分解的方法，求解方程AtA*X=AtB，得到X
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    if (iterCount == 0) {
        cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

        // 计算At*A的特征值和特征向量
        // 特征值存放在matE，特征向量matV
        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        // 退化的具体表现是指什么？
        isDegenerate = false;
        float eignThre[3] = {10, 10, 10};
        for (int i = 2; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 3; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                // 存在比10小的特征值则出现退化
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformCur[1] += matX.at<float>(0, 0);
    transformCur[3] += matX.at<float>(1, 0);
    transformCur[5] += matX.at<float>(2, 0);

    for(int i=0; i<6; i++){
        if(isnan(transformCur[i]))
            transformCur[i]=0;
    }

    float deltaR = sqrt(
                        pow(rad2deg(matX.at<float>(0, 0)), 2));
    float deltaT = sqrt(
                        pow(matX.at<float>(1, 0) * 100, 2) +
                        pow(matX.at<float>(2, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1) {
        return false;
    }
    return true;
}

//更新里程计信息
void LidarOdometry::updateTransformation(){
    if (laserCloudCornerLastNum_ < 10 || laserCloudSurfLastNum_ < 100)
        return;
    for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
        laserCloudOri_->clear();
        coeffSel_->clear();

        // 找到对应的特征平面
        // 然后计算协方差矩阵，保存在coeffSel队列中
        // laserCloudOri中保存的是对应于coeffSel的未转换到开始时刻的原始点云数据
        findCorrespondingSurfFeatures(iterCount1);

        if (laserCloudOri_->points.size() < 10)
            continue;
        // 通过面特征的匹配，计算变换矩阵
        if (calculateTransformationSurf(iterCount1) == false)
            break;
    }

    for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {

        laserCloudOri_->clear();
        coeffSel_->clear();

        // 找到对应的特征边/角点
        // 寻找边特征的方法和寻找平面特征的很类似，过程可以参照寻找平面特征的注释
        findCorrespondingCornerFeatures(iterCount2);

        if (laserCloudOri_->points.size() < 10)
            continue;
        // 通过角/边特征的匹配，计算变换矩阵
        if (calculateTransformationCorner(iterCount2) == false)
            break;
    }
}
// 累加里程计的旋转量
void LidarOdometry::accumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                            float &ox, float &oy, float &oz){
    // 参考：https://www.cnblogs.com/ReedLW/p/9005621.html
    // 0--->(cx,cy,cz)--->(lx,ly,lz)
    // 从0时刻到(cx,cy,cz),然后在(cx,cy,cz)的基础上又旋转(lx,ly,lz)
    // 求最后总的位姿结果是什么？
    // R*p_cur = R_c*R_l*p_cur  ==> R=R_c* R_l
    //
    //     |cly*clz+sly*slx*slz  clz*sly*slx-cly*slz  clx*sly|
    // R_l=|    clx*slz                 clx*clz          -slx|
    //     |cly*slx*slz-clz*sly  cly*clz*slx+sly*slz  cly*clx|
    // R_c=...
    // -srx=(ccx*scy,-scx,cly*clx)*(clx*slz,clx*clz,-slx)
    // ...
    // 然后根据R再来求(ox,oy,oz)

    float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
    ox = -asin(srx);

    float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
                    + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
    float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
                    - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
    oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

    float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
                    + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
    float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
                    - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
    oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

// 位姿矩阵的累计变化量，相对于第一帧的旋转矩阵
void LidarOdometry::integrateTransformation(){
    float rx, ry, rz, tx, ty, tz; 
    // 将计算的两帧之间的位姿“累加”起来，获得相对于第一帧的旋转矩阵
    // transformSum + (-transformCur) =(rx,ry,rz)
    accumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                        -transformCur[0], -transformCur[1], -transformCur[2], rx, ry, rz);

    // 进行平移分量的更新
    // 绕z轴旋转的恢复
    float x1 = cos(rz) * transformCur[3] 
                - sin(rz) * transformCur[4];
    float y1 = sin(rz) * transformCur[3] 
                + cos(rz) * transformCur[4];
    float z1 = transformCur[5];
    // 绕x轴旋转的恢复
    float x2 = x1;
    float y2 = cos(rx) * y1 - sin(rx) * z1;
    float z2 = sin(rx) * y1 + cos(rx) * z1;
    // 绕y轴旋转的恢复
    tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
    ty = transformSum[4] - y2;
    tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

    transformSum[0] = rx;
    transformSum[1] = ry;
    transformSum[2] = rz;
    transformSum[3] = tx;
    transformSum[4] = ty;
    transformSum[5] = tz;
}

void LidarOdometry::checkSystemInitialization(){
    // 系统初始化，将当前收到的第一帧特征点信息存为上一帧信息
    // 交换cornerPointsLessSharp和laserCloudCornerLast
    pcl::PointCloud<PointT_I>::Ptr laserCloudTemp = cornerPointsLessSharp_;
    cornerPointsLessSharp_ = laserCloudCornerLast_;
    laserCloudCornerLast_ = laserCloudTemp;

    // 交换surfPointsLessFlat和laserCloudSurfLast
    laserCloudTemp = surfPointsLessFlat_;
    surfPointsLessFlat_ = laserCloudSurfLast_;
    laserCloudSurfLast_ = laserCloudTemp;

    laserCloudCornerLastNum_ = laserCloudCornerLast_->points.size();
    laserCloudSurfLastNum_ = laserCloudSurfLast_->points.size();
    if (laserCloudCornerLastNum_ > 10 && laserCloudSurfLastNum_ > 100) {
        kdtreeCornerLast_->setInputCloud(laserCloudCornerLast_);
        kdtreeSurfLast_->setInputCloud(laserCloudSurfLast_);
    }
    systemInited = true;
}

// 发布里程计数据，将线程中计算得到的里程计传递出去
void LidarOdometry::publishOdometry(){
    OdometryOut out;
    // 输出参数，并进行zxy坐标系到xyz坐标系的转换
    out.transformDataSum[0] = transformSum[2];
    out.transformDataSum[1] = transformSum[0];
    out.transformDataSum[2] = transformSum[1];
    out.transformDataSum[3] = transformSum[5];
    out.transformDataSum[4] = transformSum[3];
    out.transformDataSum[5] = transformSum[4];
    output_channel_.push(out);
}

// 更新上一帧点云信息
void LidarOdometry::updateCloudsLast(){
    int cornerPointsLessSharpNum = cornerPointsLessSharp_->points.size();
    for (int i = 0; i < cornerPointsLessSharpNum; i++) {
        transformCloudToEnd(&cornerPointsLessSharp_->points[i],
                    &cornerPointsLessSharp_->points[i]);
    }

    int surfPointsLessFlatNum = surfPointsLessFlat_->points.size();
    for (int i = 0; i < surfPointsLessFlatNum; i++) {
        transformCloudToEnd(&surfPointsLessFlat_->points[i],
                    &surfPointsLessFlat_->points[i]);
    }

    pcl::PointCloud<PointT_I>::Ptr laserCloudTemp = cornerPointsLessSharp_;
    cornerPointsLessSharp_ = laserCloudCornerLast_;
    laserCloudCornerLast_ = laserCloudTemp;

    laserCloudTemp = surfPointsLessFlat_;
    surfPointsLessFlat_ = laserCloudSurfLast_;
    laserCloudSurfLast_ = laserCloudTemp;

    laserCloudCornerLastNum_ = laserCloudCornerLast_->points.size();
    laserCloudSurfLastNum_ = laserCloudSurfLast_->points.size();

    if (laserCloudCornerLastNum_ > 10 && laserCloudSurfLastNum_ > 100) {
        kdtreeCornerLast_->setInputCloud(laserCloudCornerLast_);
        kdtreeSurfLast_->setInputCloud(laserCloudSurfLast_);
    }
}

// 执行里程计计算程序
void LidarOdometry::runLidarOdometry(){
    while(true){
        // 接收数据
        ExtractionOut extraction;
        input_channel_.pop_uptodate(extraction);
        *cornerPointsSharp_ = extraction.cornerPointsSharp;
        *cornerPointsLessSharp_ = extraction.cornerPointsLessSharp;
        *surfPointsFlat_ = extraction.surfPointsFlat;
        *surfPointsLessFlat_ = extraction.surfPointsLessFlat;

        if(!systemInited){
            checkSystemInitialization();
            continue;
        }
        // 当前配准初始值设置为0
        for(int i=0; i<6; i++){
            transformCur[i] = 0.0;
        }
        //updateInitialPoseByImu();
        updateTransformation();
        // 积分总变换
        integrateTransformation();
        // 发布里程计数据
        publishOdometry();
        // 更新上一帧点云信息
        updateCloudsLast();
    }
}
