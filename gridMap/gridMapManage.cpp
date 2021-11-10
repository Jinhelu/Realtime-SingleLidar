#include "gridMapManage.h"

GridMapManage::GridMapManage(const GridMapParams& params) : params_(params){
    gridMap_ = GridMap(params_.n_gridmap_x, params_.n_gridmap_y);
}

// 使用ransac方法获取平面方程数据
void GridMapManage::planeMatchRANSAC(PointCloud::Ptr& pointcloud, std::vector<int>& inliers, pcl::ModelCoefficients& PlaneCoeff2Show){
    
    Eigen::VectorXf coefficients;//声明变量存储参数
    pointcloud->points.pop_back();//随机采样一致性的随机数根据点个数给出，点数不变结果不变
    //创建随机采样一致性对象
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pointcloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);

    ransac.setDistanceThreshold (params_.plane_dist_threshold);   //与平面的距离小于0.01作为局内点考虑
    ransac.computeModel();                //执行随机参数估计
    ransac.getInliers(inliers);           //存储局内点
    //ransac.setMaxIterations(6);
    //ransac.setProbability(0.85);
    ransac.getModelCoefficients(coefficients);
    //参数转换
    PlaneCoeff2Show.values.push_back(coefficients.x());
    PlaneCoeff2Show.values.push_back(coefficients.y());
    PlaneCoeff2Show.values.push_back(coefficients.z());
    PlaneCoeff2Show.values.push_back(coefficients.w());
}

//判断点在线段哪一侧      横坐标   纵坐标    端点横坐标   端点纵坐标; 上侧为 true , 下侧为false;
bool GridMapManage::LinePointJudge (double x,double y,double EndX,double EndY){
    double yflag = (x * EndY) / EndX ;
    return (y > yflag);
}
//判断线段是否经过矩形 矩形顶点 x最小值 y最小值        长       宽   过原点线段端点 x    y; 经过该矩形为true  不经过为false
bool GridMapManage::LineRecJudge (double x, double y, double deltaX, double deltaY, double EndX, double EndY){
    bool criterion_1 = LinePointJudge (x,       y,       EndX,EndY);
    bool criterion_2 = LinePointJudge (x+deltaX,y,       EndX,EndY);
    bool criterion_3 = LinePointJudge (x,       y+deltaY,EndX,EndY);
    bool criterion_4 = LinePointJudge (x+deltaX,y+deltaY,EndX,EndY);

    return !(criterion_1 == criterion_2 && criterion_3 == criterion_4 && criterion_1 == criterion_3);
}


// 根据地面点云估计地面方程参数
void GridMapManage::groundEquationEstimate(PointCloud::Ptr& pointcloud, pcl::ModelCoefficients& PlaneCoeff2Show){
    std::vector<vector<float>> estimateRes;
    int exec_estimateNum=0;
    int estimateNum = params_.n_ground_estimate;
    //多次进行随机采样, n_ground_estimate采样次数
    while(exec_estimateNum != estimateNum){
        exec_estimateNum++;
        //随机采样一致性计算出地面方程
        std::vector<int> inliers;//inliers存储结果点ID
        pcl::ModelCoefficients PlaneCoeff2Show_try;//用于显示的平面参数变量
        // 调用ransac方法估计
        this->planeMatchRANSAC(pointcloud, inliers, PlaneCoeff2Show_try);
        float c = PlaneCoeff2Show_try.values[2];
        float a = PlaneCoeff2Show_try.values[0]/c;
        float b = PlaneCoeff2Show_try.values[1]/c;
        float d = PlaneCoeff2Show_try.values[3]/c;
        c=1;
        vector<float> temp(3);
        temp[0] = atanf(a)*180/3.1415926f;//pitch角度
        temp[1] = atanf(b)*180/3.1415926f;//roll角度
        temp[2] = fabs(d/(sqrtf(a*a+b*b+1)));//面到原点距离

        estimateRes.push_back(temp);
    }
    //求平均值 pitch、roll、L的平均值
    vector<float> average_esti(3, 0.0);
    for(auto& temp:estimateRes){
        average_esti[0] += temp[0];
        average_esti[1] += temp[1];
        average_esti[2] += temp[2];
    }
    average_esti[0]/=estimateNum;
    average_esti[1]/=estimateNum;
    average_esti[2]/=estimateNum;

    //去除极值
    vector<int> removeNum;
    for(int i=0; i<estimateRes.size(); i++){
        vector<float> temp(3);
        temp[0] = estimateRes[i][0];
        temp[1] = estimateRes[i][1];
        temp[2] = estimateRes[i][2];

        if(temp[0] > params_.max_deviation_deg || temp[1] > params_.max_deviation_deg) {
            removeNum.push_back(i);
        }
    }
    for(int i=0; i<removeNum.size(); i++){
        estimateRes.erase(estimateRes.begin()+removeNum[i]);
    }
    //再求平均值
    vector<float> average_esti_2(3);
    int newEstimateNum=0;
    for(auto temp:estimateRes){
        average_esti_2[0] += temp[0];
        average_esti_2[1] += temp[1];
        average_esti_2[2] += temp[2];

        newEstimateNum++;
    }

    //确定最终的pitch、roll、L估计值
    if(newEstimateNum){
        average_esti_2[0]/=newEstimateNum;
        average_esti_2[1]/=newEstimateNum;
        average_esti_2[2]/=newEstimateNum;

        average_esti = average_esti_2;
        // if(newEstimateNum >= estimateNum/5) precise = true;
        // else {
		// 				//cout << "Ground Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
		// 		}
    }
    else {
        //cout << "Ground Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
    }
    //最终的pitch、roll、L估计值转换为 平面参数
    float c_esti = 1;
    float a_esti = tanf((average_esti[0]*3.1415926f)/180)*c_esti;
    float b_esti = tanf((average_esti[1]*3.1415926f)/180)*c_esti;
    float d_esti = average_esti[2]*(sqrt(a_esti*a_esti + b_esti*b_esti+1));

    PlaneCoeff2Show.values.clear();
    PlaneCoeff2Show.values.push_back(a_esti);
    PlaneCoeff2Show.values.push_back(b_esti);
    PlaneCoeff2Show.values.push_back(c_esti);
    PlaneCoeff2Show.values.push_back(d_esti);
}

void GridMapManage::groundCastFilter(const PointCloud::Ptr& inputCloud, pcl::ModelCoefficients& coefficients,
                       const PointCloud::Ptr& outputCloud){
    //平面方程 ax+by+cz+d=0
    pcl::ModelCoefficients::Ptr coefficientsPtr (new pcl::ModelCoefficients ());
    coefficientsPtr->values.resize (4);
    coefficientsPtr->values = coefficients.values;

    //投影的初始化,
    //TODO 是不是去除地面平面以后，提取到的其他点云
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (inputCloud);
    proj.setModelCoefficients (coefficientsPtr);
    proj.filter (*outputCloud);
}

//将投影的平面点云转换为栅格地图
void GridMapManage::planeCloud2Gridmap(const PointCloud::Ptr& map_base_plane){
    double deltaX = params_.grid_scale;       //每个删格的X轴分辨率 单位：米
    double deltaY = params_.grid_scale;      //每个删格的Y轴分辨率 单位：米
    //对所有点进行分析
    for (int i=0; i<map_base_plane->points.size(); i++) {
        //对scope范围内的点进行分析,雷达中的点一定是占位点，是1
        if (map_base_plane -> points[i].x < params_.max_x && map_base_plane -> points[i].x > params_.min_x  &&
            map_base_plane -> points[i].y < params_.max_y && map_base_plane -> points[i].y > params_.min_y) {

            int End_XGrid_No, End_YGrid_No;
            int LineGridNumX = abs(int(map_base_plane->points[i].x/deltaX));
            int LineGridNumY = abs(int(map_base_plane->points[i].y/deltaY));  //Y坐标删格整数（不带负号）

            //提取坐标的符号 正号为1 负号为-1
            int SignDataX, SignDataY;
            if (map_base_plane->points[i].x > 0){ SignDataX=1; }
            else { SignDataX = -1; }
            if (map_base_plane->points[i].y > 0){ SignDataY=1; }
            else { SignDataY = -1; }

            //////////////////////////////端点删格判断///////////////////////////////////////
            if (map_base_plane->points[i].x < 0){
                End_XGrid_No = SignDataX*LineGridNumX + (gridMap_.getLaserNumX() + 1); }
            else{ End_XGrid_No = SignDataX*LineGridNumX + (gridMap_.getLaserNumX() + 1) + 1; }

            if (map_base_plane->points[i].y < 0){
                End_YGrid_No = (-SignDataY)*LineGridNumY +(gridMap_.getLaserNumY() + 1) + 1; }
            else{ End_YGrid_No = (-SignDataY)*LineGridNumY +(gridMap_.getLaserNumY() + 1); }

            //更新删格状态
            gridMap_.modifyGridStateinLaserCoord(End_XGrid_No, End_YGrid_No, OCCU);
        }
    }
    //对激光无反射回来值的点，添加到点云中，认为其在无穷远处返回  点的间隔为gridScale/2
    // 人为限制点云的反射范围
    float tempZ = map_base_plane -> points[0].z;
    float tempNumber;
    //填充极限params_.max_y
    tempNumber = 2;
    while(tempNumber < params_.max_x){
        tempNumber += params_.grid_scale;
        PointT tempPoint(tempNumber, float(params_.max_y - 0.2), tempZ);
        map_base_plane->points.push_back(tempPoint);
    }
    //填充极限params_.min_y
    tempNumber = 2;
    while(tempNumber < params_.max_x){
        tempNumber += params_.grid_scale;
        PointT tempPoint(tempNumber, float(params_.min_y + 0.2), tempZ);
        map_base_plane->points.push_back(tempPoint);
    }
    //填充极限params_.max_x
    tempNumber = (float)params_.min_y;
    while(tempNumber < (float)params_.max_y){
        tempNumber += params_.grid_scale;
        PointT tempPoint(float(params_.max_x-0.2),tempNumber,tempZ);
        map_base_plane->points.push_back(tempPoint);
    }



    //对删格的所有点进行占位判断后再进行激光束经过判断
    //防止造成后面点对前面点的干扰,出现两个障碍物之间的区域为未占用的情况
    //对所有点进行分析
    for (int i=0; i<map_base_plane->points.size(); i++){
        //对scope范围内的点进行分析
        if (map_base_plane -> points[i].x < params_.max_x && map_base_plane -> points[i].x > params_.min_x  &&
            map_base_plane -> points[i].y < params_.max_y && map_base_plane -> points[i].y > params_.min_y  ) {

            int End_XGrid_No,End_YGrid_No;
            int LineGridNumX = abs(int(map_base_plane->points[i].x/deltaX) );
            int LineGridNumY = abs(int(map_base_plane->points[i].y/deltaY) );  //Y坐标删格整数（不带fuhao）

            ////提取坐标的符号 正号为1 负号为-1
            int SignDataX,SignDataY;
            if (map_base_plane->points[i].x > 0){SignDataX=1;}
            else {SignDataX=-1;}
            if (map_base_plane->points[i].y > 0){SignDataY=1;}
            else {SignDataY=-1;}

            /////////////////////////////从原点开始追溯到端点///////////////////////////////////
            //当前判断删格横坐标 X  删格横坐标 X    预判断X    预判断Y
            int startNumX, startNumY, judgeNumX, judgeNumY;

            //对象限的分析确定最初始的实际point原点附近的删格坐标
            if(SignDataX == 1) {startNumX = (gridMap_.getLaserNumX() + 1) + 1;}
            else {startNumX = (gridMap_.getLaserNumX() + 1);}
            if(SignDataY == 1) {startNumY = (gridMap_.getLaserNumY() + 1);}
            else {startNumY = (gridMap_.getLaserNumY() + 1) + 1;}

            //向前追溯
            for(int j=0; j<LineGridNumX+LineGridNumY; j++){
                //如果追溯到已经占位的删格点则停止,后面的删格必然是未知的
                if (gridMap_.getGridState(startNumX,startNumY)==UNKNOW ){
                    gridMap_.ModifyGridState(startNumX,startNumY,FREE);
                }
                else if (gridMap_.getGridState(startNumX,startNumY)==OCCU){
                    break;
                }

                //向X方向向前一步进行预判
                judgeNumX = startNumX+SignDataX;
                judgeNumY = startNumY;

                //预判删格的左下角point坐标
                double judgeRecX = (judgeNumX - (gridMap_.getLaserNumX() + 1) -1)*deltaX;
                double judgeRecY = -(judgeNumY - (gridMap_.getLaserNumY() + 1))*deltaX;

                //判断该删格是否被经过
                // TODO 大坑
                bool flag = LineRecJudge (judgeRecX,judgeRecY,deltaX,deltaY,
                                        map_base_plane->points[i].x,map_base_plane->points[i].y);

                //如果该删格被经过,该删格作为现在判断点
                if(flag) startNumX=judgeNumX;
                //如果该删格未被经过,向Y方向试探作为现在判断点
                else startNumY = startNumY - SignDataY;
            }
        }
    }
    //最后将机器人本体近邻的栅格设置为FREE状态，防止程序卡死
    double safeDistance = 1.1;//1.1m安全距离
    int safeGridNum = int(safeDistance/params_.grid_scale);

    for(int i = -safeGridNum; i < safeGridNum; i++){
        for(int j = -safeGridNum; j < safeGridNum; j++){
        //TODO：grid
        if(gridMap_.getGridStateinLaserCoord(i,j) == UNKNOW )
            gridMap_.modifyGridStateinLaserCoord(i, j, FREE);
        }
    }
}

//将目标点真实雷达坐标转换到栅格地图中
void GridMapManage::RealCoord2GridCoord(float x, float y, int &coloum_grid, int &row_grid){
    int LineGridNumX = abs(int(x/params_.n_pixel_per_grid));
    int LineGridNumY = abs(int(y/params_.n_pixel_per_grid));  //Y坐标删格整数（不带fuhao）

    // 提取坐标的符号 正号为1 负号为-1
    int SignDataX,SignDataY;
    if (x > 0) SignDataX = 1; 
    else SignDataX = -1;
    if (y > 0) SignDataY = 1;
    else SignDataY = -1;

    // 目标点删格位置判断
    if (x < 0)
        coloum_grid = (SignDataX)*LineGridNumX + (gridMap_.getLaserNumX() + 1);
    else
        coloum_grid = (SignDataX)*LineGridNumX + (gridMap_.getLaserNumX() + 1) + 1; 

    if (y < 0)
        row_grid = (-SignDataY)*LineGridNumY + (gridMap_.getLaserNumY() + 1) + 1;
    else
        row_grid = (-SignDataY)*LineGridNumY + (gridMap_.getLaserNumY() + 1);
}

cv::Mat GridMapManage::gridMap2Mat(){
    //地图总像素大小
    int delta = params_.n_pixel_per_grid; // 每条栅格边占据的像素个数
    int width = gridMap_.getWidthNum()*delta;
    int height = gridMap_.getHeightNum()*delta;

    cv::Mat MapImage(width, height, CV_8UC1, cv::Scalar(180));//创建原始图像
    for(int j=0; j<gridMap_.getHeightNum(); j++){
        //对每一行y
        for(int i=0; i<gridMap_.getWidthNum(); i++){
        //对每行的每个删格x
            if (gridMap_.getGridState(i,j) == FREE){
            //空闲状态为白色
            //画矩形（图像，一个顶点，对角顶点，颜色，线条粗细，线条类型）
            cv::rectangle(MapImage,
                        cv::Point(i*delta,j*delta), cv::Point(i*delta+delta,j*delta+delta),
                        cv::Scalar(255),-1,8);
            }
            else if(gridMap_.getGridState(i,j) == OCCU){
            //占用状态为黑色
            cv::rectangle(MapImage,
                        cv::Point(i*delta,j*delta), cv::Point(i*delta+delta,j*delta+delta),
                        cv::Scalar(0),-1,8);
            }
        }//对每行的每个删格x
    }//对每一行y
    return MapImage;
}