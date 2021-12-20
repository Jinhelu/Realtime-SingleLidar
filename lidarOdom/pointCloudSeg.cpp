#include "pointCloudSeg.h"

PointCloudSeg::PointCloudSeg(const InitParams& params){
    groundSegTool_ = GroundSegmentation(params);
    variablesInit();
}
 
// 类成员变量初始化
void PointCloudSeg::variablesInit(){
    laserCloudIn_.reset(new pcl::PointCloud<PointT_I>());
    orderedFullCloud_.reset(new pcl::PointCloud<PointT_I>());
    fullInfoCloud_.reset(new pcl::PointCloud<PointT_I>());

    orderedFullCloud_->points.resize(N_SCAN*Horizon_SCAN);
    fullInfoCloud_->points.resize(N_SCAN*Horizon_SCAN);

    groundCloud_.reset(new pcl::PointCloud<PointT_I>());
    segmentedCloud_.reset(new pcl::PointCloud<PointT_I>());
    cloudforGroundSeg_.reset(new pcl::PointCloud<PointT>());

    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    // 该矩阵用于求某个点的上下左右4个邻接点
    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
    neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
    neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);
    
    // 用于BFS聚类的搜索
    allPushedIndX = new int[N_SCAN*Horizon_SCAN];
    allPushedIndY = new int[N_SCAN*Horizon_SCAN];
    queueIndX = new int[N_SCAN*Horizon_SCAN];
    queueIndY = new int[N_SCAN*Horizon_SCAN];
}
void PointCloudSeg::resetVariables(){
    laserCloudIn_->clear();
    groundCloud_->clear();
    segmentedCloud_->clear();
    cloudforGroundSeg_->clear();
    segmentedCloud_->clear();

    rangeMat_ = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat_ = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat_ = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    labelCount_ = 1;
    for(size_t i=0; i<N_SCAN; i++){
        for(size_t j=0; j<Horizon_SCAN; j++){
            depthMat_[i][j] = 0;
            indexMat_[i][j] = -1;
            groundSegMat_[i][j] = 0;
            verticalFlagMat_[i][j] = false;
        }
    }
    std::fill(orderedFullCloud_->points.begin(), orderedFullCloud_->points.end(), nanPoint);
    std::fill(fullInfoCloud_->points.begin(), fullInfoCloud_->points.end(), nanPoint);
}

// lego-slam的重投影及分割聚类
void PointCloudSeg::cloudHandler(const PointCloud_I::Ptr& laserCloudIn){
    resetVariables();
    *laserCloudIn_ = *laserCloudIn;
    unorderedCloud2Structure(laserCloudIn_);
    groundRemoval();
    cloudSegmentation();
}

// 点云数据结构化
void PointCloudSeg::unorderedCloud2Structure(const PointCloud_I::Ptr& laserCloudIn){
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize; 
    PointT_I thisPoint;

    cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i){
        if(laserCloudIn->points[i].intensity == -1) continue;

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;

        // 计算竖直方向上的角度（雷达的第几线）
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        
        // rowIdn计算出该点激光雷达是竖直方向上第几线的
        // 从下往上计数，-15度记为初始线，第0线，一共16线(N_SCAN=16)
        if(N_SCAN == 16){
            //rowIdn = (verticalAngle + 15) / ang_res_y; 
            rowIdn = int((verticalAngle + 15) / 2 + 0.5); 
        }else if(N_SCAN == 32){
            rowIdn = int((verticalAngle + 92.0/3.0) * 3.0 / 4.0);
        }
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;  

        // atan2(y,x)函数的返回值范围(-PI,PI],表示与复数x+yi的幅角
        // 下方角度atan2(..)交换了x和y的位置，计算的是与y轴正方向的夹角大小(关于y=x做对称变换)
        // 这里是在雷达坐标系，所以是与正前方的夹角大小
        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        // round函数进行四舍五入取整
        // 这边确定不是减去180度???  不是
        // 雷达水平方向上某个角度和水平第几线的关联关系???关系如下：
        // horizonAngle:(-PI,PI],columnIdn:[H/4,5H/4]-->[0,H] (H:Horizon_SCAN)
        // 下面是把坐标系绕z轴旋转,对columnIdn进行线性变换
        // x+==>Horizon_SCAN/2,x-==>Horizon_SCAN
        // y+==>Horizon_SCAN*3/4,y-==>Horizon_SCAN*5/4,Horizon_SCAN/4
        //
        //          3/4*H
        //          | y+
        //          |
        // (x-)H---------->H/2 (x+)
        //          |
        //          | y-
        //    5/4*H   H/4
        //
        columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;
        // 经过上面columnIdn -= Horizon_SCAN的变换后的columnIdn分布：
        //          3/4*H
        //          | y+
        //     H    |
        // (x-)---------->H/2 (x+)
        //     0    |
        //          | y-
        //         H/4
        //
        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        rangeMat_.at<float>(rowIdn, columnIdn) = range;

        // columnIdn:[0,H] (H:Horizon_SCAN)==>[0,1800]
        thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

        index = columnIdn  + rowIdn * Horizon_SCAN;
        orderedFullCloud_->points[index] = thisPoint;

        fullInfoCloud_->points[index].intensity = range;
    }
}

// labelMat的BFS遍历进行标记聚类(连通图算法)
void PointCloudSeg::labelComponents(int row, int col){
    float d1, d2, alpha, angle;
    int fromIndX, fromIndY, thisIndX, thisIndY; 
    bool lineCountFlag[N_SCAN] = {false};

    queueIndX[0] = row;
    queueIndY[0] = col;
    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    allPushedIndX[0] = row;
    allPushedIndY[0] = col;
    int allPushedIndSize = 1;
    
    // BFS的作用是以(row，col)为中心向外面扩散，
    // 判断(row,col)是否是这个平面中一点
    while(queueSize > 0){
        fromIndX = queueIndX[queueStartInd];
        fromIndY = queueIndY[queueStartInd];
        --queueSize;
        ++queueStartInd;
        // labelCount的初始值为1，后面会递增
        labelMat_.at<int>(fromIndX, fromIndY) = labelCount_;

        // neighbor=[[-1,0];[0,1];[0,-1];[1,0]]
        // 遍历点[fromIndX,fromIndY]边上的四个邻点
        for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){

            thisIndX = fromIndX + (*iter).first;
            thisIndY = fromIndY + (*iter).second;

            if (thisIndX < 0 || thisIndX >= N_SCAN)
                continue;

            // 是个环状的图片，左右连通
            if (thisIndY < 0)
                thisIndY = Horizon_SCAN - 1;
            if (thisIndY >= Horizon_SCAN)
                thisIndY = 0;

            // 如果点[thisIndX,thisIndY]已经标记过
            // labelMat中，-1代表无效点，0代表未进行标记过，其余为其他的标记
            // 如果当前的邻点已经标记过，则跳过该点。
            // 如果labelMat已经标记为正整数，则已经聚类完成，不需要再次对该点聚类
            if (labelMat_.at<int>(thisIndX, thisIndY) != 0)
                continue;

            d1 = std::max(rangeMat_.at<float>(fromIndX, fromIndY), 
                            rangeMat_.at<float>(thisIndX, thisIndY));
            d2 = std::min(rangeMat_.at<float>(fromIndX, fromIndY), 
                            rangeMat_.at<float>(thisIndX, thisIndY));

            // alpha代表角度分辨率，
            // X方向上角度分辨率是segmentAlphaX(rad)
            // Y方向上角度分辨率是segmentAlphaY(rad)
            if ((*iter).first == 0)
                alpha = segmentAlphaX;
            else
                alpha = segmentAlphaY;

            // 通过下面的公式计算这两点之间是否有平面特征
            // atan2(y,x)的值越大，d1，d2之间的差距越小,越平坦
            angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

            if (angle > segmentTheta){
                // segmentTheta=1.0472<==>60度
                // 如果算出角度大于60度，则假设这是个平面
                queueIndX[queueEndInd] = thisIndX;
                queueIndY[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;

                labelMat_.at<int>(thisIndX, thisIndY) = labelCount_;
                lineCountFlag[thisIndX] = true;

                allPushedIndX[allPushedIndSize] = thisIndX;
                allPushedIndY[allPushedIndSize] = thisIndY;
                ++allPushedIndSize;
            }
        }
    }
    bool feasibleSegment = false;
    // 如果聚类超过30个点，直接标记为一个可用聚类，labelCount需要递增
    if (allPushedIndSize >= 30)
        feasibleSegment = true;
    else if (allPushedIndSize >= segmentValidPointNum){
        // 如果聚类点数小于30大于等于5，统计竖直方向上的聚类点数
        int lineCount = 0;
        for (int i = 0; i < N_SCAN; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;

        // 竖直方向上超过3个也将它标记为有效聚类
        if (lineCount >= segmentValidLineNum)
            feasibleSegment = true;            
    }

    if (feasibleSegment == true){
        ++labelCount_;
    }else{
        for (int i = 0; i < allPushedIndSize; ++i){
            // 标记为999999的是需要舍弃的聚类的点，因为他们的数量小于30个
            labelMat_.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
        }
    }
}

void PointCloudSeg::groundRemoval(){
    XYZI2XYZ(orderedFullCloud_, cloudforGroundSeg_);
    groundSegTool_.segment(*cloudforGroundSeg_, &groundLabel_);
    // 保存地面点云
    for(size_t i = 0; i < orderedFullCloud_->size(); i++){
        if (groundLabel_[i] == 1){
            groundCloud_->push_back(orderedFullCloud_->points.at(i));
            int rowIdn = (int)orderedFullCloud_->points.at(i).intensity;
	        int columnIdn = (int)((orderedFullCloud_->points.at(i).intensity - rowIdn)*10000);
            groundMat_.at<int8_t>(rowIdn, columnIdn) = 1;
        }
    }
    // 找到所有点中的地面点或者距离为FLT_MAX(rangeMat的初始值)的点，并将他们标记为-1
    // rangeMat[i][j]==FLT_MAX，表示无效点
    for (size_t i = 0; i < N_SCAN; ++i){
        for (size_t j = 0; j < Horizon_SCAN; ++j){
            if (groundMat_.at<int8_t>(i,j) == 1 || rangeMat_.at<float>(i,j) == FLT_MAX){
                labelMat_.at<int>(i,j) = -1;
            }
        }
    }
}

void PointCloudSeg::cloudSegmentation(){
    for (size_t i = 0; i < N_SCAN; ++i)
        for (size_t j = 0; j < Horizon_SCAN; ++j)
            // 如果labelMat[i][j]=0,表示没有对该点进行过分类,需要对该点进行聚类
            if (labelMat_.at<int>(i,j) == 0)
                labelComponents(i, j);
    int sizeOfSegCloud = 0;
    for (size_t i = 0; i < N_SCAN; ++i) {
        segMsg_.startRingIndex[i] = sizeOfSegCloud-1+5;// 从第5列开始，保证计算左右各5个点时还在同一scan
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            // 找到可用的特征点或者地面点(不选择labelMat[i][j]=0的点)
            if (labelMat_.at<int>(i,j) > 0 || groundMat_.at<int8_t>(i,j) == 1){
                if (labelMat_.at<int>(i,j) == 999999){
                    continue;
                }
                // 如果是地面点,对于列数不为5的倍数的，直接跳过不处理，减少点云处理个数
                // 因为地面点较多，出于特征点提取的均匀性考虑，并不需要连续相邻的地面点
                if (groundMat_.at<int8_t>(i,j) == 1){
                    if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                        continue;
                    segMsg_.segmentedCloudGroundFlag[sizeOfSegCloud] = 1;
                }
                segmentedCloud_->push_back(orderedFullCloud_->points[j + i*Horizon_SCAN]);
                segMsg_.segmentedCloudRange[sizeOfSegCloud] = rangeMat_.at<float>(i, j);
                segMsg_.segmentedCloudColInd[sizeOfSegCloud] = j;
                sizeOfSegCloud++;
            }
        }
        segMsg_.endRingIndex[i] = sizeOfSegCloud-1-5;// 从倒数第5列结束，保证计算左右各5个点时还在同一scan
    }
}

void PointCloudSeg::publishSegInfo(PointCloud_I::Ptr& laserCloudOut, SegInfo& segInfoOut){
    *laserCloudOut = *segmentedCloud_;
    segInfoOut = segMsg_;
}