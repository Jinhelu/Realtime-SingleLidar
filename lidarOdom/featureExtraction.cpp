#include "featureExtraction.h"

FeatureExtraction::FeatureExtraction(Channel<ExtractionOut>& output_channel): 
        output_channel_(output_channel) {

    initializationValue();
    // // 处理接收输入参数
    // segmentedCloud_ = segmentedCloud;
    // for(size_t i = 0; i < segInfo.segmentedCloudRange.size(); i++){
    //     if(segInfo.segmentedCloudRange[i] > 0.0){
    //         segMsg_.segmentedCloudRange.push_back(segInfo.segmentedCloudRange[i]);
    //     }
    //     if(segInfo.segmentedCloudColInd[i] > -1){
    //         segMsg_.segmentedCloudColInd.push_back(segInfo.segmentedCloudColInd[i]);
    //     }
    // }
    // for(size_t i = 0; i < N_SCAN; i++){
    //     segMsg_.startRingIndex[i] = segInfo.startRingIndex[i];
    //     segMsg_.endRingIndex[i] = segInfo.endRingIndex[i];
    // }
}

// 初始化成员变量参数
void FeatureExtraction::initializationValue(){
     // 下采样滤波器设置叶子间距，就是格子之间的最小距离
    downSizeFilter_.setLeafSize(0.2, 0.2, 0.2);

    segmentedCloud_.reset(new pcl::PointCloud<PointT_I>());

    cornerPointsSharp_.reset(new pcl::PointCloud<PointT_I>());
    cornerPointsLessSharp_.reset(new pcl::PointCloud<PointT_I>());
    surfPointsFlat_.reset(new pcl::PointCloud<PointT_I>());
    surfPointsLessFlat_.reset(new pcl::PointCloud<PointT_I>());

    surfPointsLessFlatScan_.reset(new pcl::PointCloud<PointT_I>());
    surfPointsLessFlatScanDS_.reset(new pcl::PointCloud<PointT_I>());

    cloudSmoothness_.resize(N_SCAN * Horizon_SCAN);
}

// 雷达坐标系到运动坐标系的变换
void FeatureExtraction::coordinateTransform(){
    PointT_I point;
    for(size_t i=0; i<segmentedCloud_->points.size(); i++){
        point.x = segmentedCloud_->points[i].y;
        point.y = segmentedCloud_->points[i].z;
        point.z = segmentedCloud_->points[i].x;
        point.intensity = segmentedCloud_->points[i].intensity;
        segmentedCloud_->points[i] = point;
    }
}

// 计算平滑程度
void FeatureExtraction::calculateSmoothness(){
    int cloudSize = segmentedCloud_->points.size();
    for(int i=0; i<cloudSize-5; i++){
        float diffRange = segMsg_.segmentedCloudRange[i-5] + segMsg_.segmentedCloudRange[i-4]
                        + segMsg_.segmentedCloudRange[i-3] + segMsg_.segmentedCloudRange[i-2]
                        + segMsg_.segmentedCloudRange[i-1] - segMsg_.segmentedCloudRange[i] * 10
                        + segMsg_.segmentedCloudRange[i+1] + segMsg_.segmentedCloudRange[i+2]
                        + segMsg_.segmentedCloudRange[i+3] + segMsg_.segmentedCloudRange[i+4]
                        + segMsg_.segmentedCloudRange[i+5];
        cloudCurvature_[i] = diffRange * diffRange;
        cloudNeighborPicked_[i] = 0;
        // 在extractFeatures()函数中会对标签进行修改，初始化为0.
        cloudLabel_[i] = 0;
        cloudSmoothness_[i].value = cloudCurvature_[i];
        cloudSmoothness_[i].ind = i;  
    }
}

// 标记点云中相互遮挡又靠的很近的点
void FeatureExtraction::markOccludePoints(){
    int cloudSize = segmentedCloud_->size();
    for(int i=0; i<cloudSize-6; i++){
        float depth1 = segMsg_.segmentedCloudRange[i];
        float depth2 = segMsg_.segmentedCloudRange[i+1];
        int columnDiff = std::abs(int(segMsg_.segmentedCloudColInd[i+1] - segMsg_.segmentedCloudColInd[i]));

        if (columnDiff < 10){
            // 选择距离较远的那些点，并将他们标记为1
            if (depth1 - depth2 > 0.3){
                cloudNeighborPicked_[i - 5] = 1;
                cloudNeighborPicked_[i - 4] = 1;
                cloudNeighborPicked_[i - 3] = 1;
                cloudNeighborPicked_[i - 2] = 1;
                cloudNeighborPicked_[i - 1] = 1;
                cloudNeighborPicked_[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                cloudNeighborPicked_[i + 1] = 1;
                cloudNeighborPicked_[i + 2] = 1;
                cloudNeighborPicked_[i + 3] = 1;
                cloudNeighborPicked_[i + 4] = 1;
                cloudNeighborPicked_[i + 5] = 1;
                cloudNeighborPicked_[i + 6] = 1;
            }
        }
        float diff1 = std::abs(segMsg_.segmentedCloudRange[i-1] - segMsg_.segmentedCloudRange[i]);
        float diff2 = std::abs(segMsg_.segmentedCloudRange[i+1] - segMsg_.segmentedCloudRange[i]);
        // 选择距离变化较大的点，并将他们标记为1
        if (diff1 > 0.02 * segMsg_.segmentedCloudRange[i] && diff2 > 0.02 * segMsg_.segmentedCloudRange[i])
            cloudNeighborPicked_[i] = 1;
    }
}

// 提取四种特征点
void FeatureExtraction::extractFeatures(){
    cornerPointsSharp_->clear();
    cornerPointsLessSharp_->clear();
    surfPointsFlat_->clear();
    surfPointsLessFlat_->clear();
    for(size_t i=0; i<N_SCAN; i++){
        surfPointsLessFlatScan_->clear();
        // 每一个scan分6段进行特征点采集(保证特征点的均匀性)
        for(size_t j=0; j<6; j++){
            // 每一段起始点和终点在分割点云中的索引位置
            int startPointIdx = (segMsg_.startRingIndex[i] * (6 - j) + segMsg_.endRingIndex[i] * j) / 6;
            int endPointIdx = (segMsg_.startRingIndex[i] * (5 - j)    + segMsg_.endRingIndex[i] * (j + 1)) / 6 - 1;
            if(startPointIdx > endPointIdx) continue;
            
            // 按照cloudSmoothness.value从小到大排序, 将曲率按照从小到达排序
            std::sort(cloudSmoothness_.begin()+startPointIdx, cloudSmoothness_.begin()+endPointIdx, by_value());
            
            // 开始选择曲率大的点
            int largestPickedNum = 0;
            for (int k = endPointIdx; k >= startPointIdx; k--) {
                // 按曲率从大到小顺序，取出对应的点云
                int ind = cloudSmoothness_[k].ind;
                // 判断条件：未排除的点 && 曲率大于角点阈值的点 && 非地面点
                if (cloudNeighborPicked_[ind] == 0 &&
                    cloudCurvature_[ind] > edgeThreshold &&
                    segMsg_.segmentedCloudGroundFlag[ind] != 1) {
                    largestPickedNum++;
                    if (largestPickedNum <= 2) {
                        cloudLabel_[ind] = 2;
                        cornerPointsSharp_->push_back(segmentedCloud_->points[ind]);
                        cornerPointsLessSharp_->push_back(segmentedCloud_->points[ind]);
                    } else if (largestPickedNum <= 20) {
                        // 放20个点到cornerPointsLessSharp中去，cornerPointsLessSharp标记为1
                        cloudLabel_[ind] = 1;
                        cornerPointsLessSharp_->push_back(segmentedCloud_->points[ind]);
                    } else {
                        break;
                    }
                    cloudNeighborPicked_[ind] = 1;
                    // 防止特征点靠得太近，每次选出特征点后将该点后面5个点标记成已经被选择
                    for (int l = 1; l <= 5; l++) {
                        // 从ind+l开始后面5个点，每个点index之间的差值，
                        // 确保columnDiff<=10,然后标记为我们需要的点
                        int columnDiff = std::abs(int(segMsg_.segmentedCloudColInd[ind + l] - segMsg_.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                    // 前面5个点标记成已经被选择
                    for (int l = -1; l >= -5; l--) {
                        // 从ind+l开始前面五个点，计算差值然后标记
                        int columnDiff = std::abs(int(segMsg_.segmentedCloudColInd[ind + l] - segMsg_.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                }
            }
            // 开始选择曲率小的点
            int smallestPickedNum = 0;
            for (int k = startPointIdx; k <= endPointIdx; k++) {
                int ind = cloudSmoothness_[k].ind;
                // 判断条件：未排除的点 && 曲率小于平面点阈值的点 && 地面点
                if (cloudNeighborPicked_[ind] == 0 &&
                    cloudCurvature_[ind] < surfThreshold &&
                    segMsg_.segmentedCloudGroundFlag[ind] == 1) {
                    cloudLabel_[ind] = -1;
                    surfPointsFlat_->push_back(segmentedCloud_->points[ind]);
                    // 将4个最平的平面点放入队列中
                    smallestPickedNum++;
                    if (smallestPickedNum >= 4) {
                        break;
                    }
                    cloudNeighborPicked_[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        // 从前面往后判断是否是需要的邻接点，是的话就进行标记
                        int columnDiff = std::abs(int(segMsg_.segmentedCloudColInd[ind + l] - segMsg_.segmentedCloudColInd[ind + l - 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        // 从后往前开始标记
                        int columnDiff = std::abs(int(segMsg_.segmentedCloudColInd[ind + l] - segMsg_.segmentedCloudColInd[ind + l + 1]));
                        if (columnDiff > 10)
                            break;
                        cloudNeighborPicked_[ind + l] = 1;
                    }
                }
            }
            // 其余的点存入surfPointsLessFlatScan_中
            for (int k = startPointIdx; k <= endPointIdx; k++) {
                if (cloudLabel_[k] <= 0) {
                    surfPointsLessFlatScan_->push_back(segmentedCloud_->points[k]);
                }
            }
        }
        
        // surfPointsLessFlatScan_中有过多的点云, 进行降采样，减少计算量
        surfPointsLessFlatScanDS_->clear();
        downSizeFilter_.setInputCloud(surfPointsLessFlatScan_);
        downSizeFilter_.filter(*surfPointsLessFlatScanDS_);
        // 降采样后的作为surfPointsLessFlat_点
        *surfPointsLessFlat_ += *surfPointsLessFlatScanDS_;
    }
}

// 发布四种特征点云信息
void FeatureExtraction::publishFeatureCloud(){
    ExtractionOut extraction;

    extraction.cornerPointsLessSharp = *cornerPointsLessSharp_;
    extraction.cornerPointsSharp = *cornerPointsSharp_;
    extraction.surfPointsFlat = *surfPointsFlat_;
    extraction.surfPointsLessFlat = *surfPointsLessFlat_;

    output_channel_.push(extraction);
}

void FeatureExtraction::runFeatureAssociation(const pcl::PointCloud<PointT_I>::Ptr segmentedCloud,  SegInfo& segInfo){
    // 接收输入参数
    *segmentedCloud_ = *segmentedCloud;
    segMsg_ = segInfo; 
    // 进行雷达坐标系到运动坐标系的变换
    coordinateTransform();
    // // 主要进行的处理是将点云数据进行坐标变换，进行插补等工作,畸变矫正
    // adjustDistortion(); 
    // 不完全按照公式进行光滑性计算，并保存结果
    calculateSmoothness();

    // 去除过近的点，在点云中可能出现的互相遮挡的情况
    markOccludePoints();

    // 特征抽取，然后分别保存到cornerPointsSharp等等队列中去
    // 保存到不同的队列是不同类型的点云，进行了标记的工作，
    // 这一步中减少了点云数量，使计算量减少
    extractFeatures();

    // 发布cornerPointsSharp等4种类型的点云数据
    publishFeatureCloud();
}
