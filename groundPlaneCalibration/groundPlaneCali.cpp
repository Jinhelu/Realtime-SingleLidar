#include "groundPlaneCali.h"

GroundPlaneCali::GroundPlaneCali(const InitParams& params){
    // 参数赋值
    params_.groundCali_deg_threshold = params.groundCali_deg_threshold;
    params_.max_deviation_deg = params.max_deviation_deg;
    params_.n_ground_estimate = params.n_ground_estimate;
    params_.plane_dist_threshold = params.plane_dist_threshold;
}

// planeMatchRANSAC使用ransac方法获取平面方程数据
void GroundPlaneCali::planeMatchRANSAC(PointCloud::Ptr& pointcloud, std::vector<int>& inliers, pcl::ModelCoefficients& PlaneCoeff2Show){
    Eigen::VectorXf coefficients;//声明变量存储参数
    pointcloud->points.pop_back();//随机采样一致性的随机数根据点个数给出，点数不变结果不变
    //创建随机采样一致性对象
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
            model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (pointcloud));// 针对平面模型对象

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

// groundEquationEstimate 获取地面估计的方程 ax+by+cz+d=0
void GroundPlaneCali::groundEquationEstimate(PointCloud::Ptr& pointcloud, pcl::ModelCoefficients& PlaneCoeff2Show, bool& needGroundCali){
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
        temp[0] = atanf(a)*180/M_PI;//pitch角度
        temp[1] = atanf(b)*180/M_PI;//roll角度
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
    }
    else {
        //cout << "Ground Plane is not precise!!!!!!!!!!!!!!!!!!!!!!!" <<endl;
    }
    // 判断是否进行地面矫正
    if(average_esti[0] > params_.groundCali_deg_threshold || average_esti[1] > params_.groundCali_deg_threshold){
        needGroundCali = true;
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

// getCaliRotateMatrix 对估计地面进行地平面校准，获取旋转矩阵
Eigen::Matrix3f GroundPlaneCali::getCaliRotateMatrix(pcl::ModelCoefficients& PlaneCoeff2Show){
    //生成地面估计的法向量
    float a = PlaneCoeff2Show.values[0];
    float b = PlaneCoeff2Show.values[1];
    float c = PlaneCoeff2Show.values[2];
    Eigen::Vector3f groundEstimateNormal(a, b, c);
    // 获取旋转矩阵
    Eigen::Matrix3f rotationMatrix = CreateRotateMatrix(groundEstimateNormal);
    return rotationMatrix;
}

// CreateRotateMatrix 计算两个向量间的旋转矩阵
Eigen::Matrix3f GroundPlaneCali::CreateRotateMatrix(Eigen::Vector3f& groundEstimateNormal){
    // 输入groundEstimateNormal是旋转后的法向量
    Eigen::Vector3f standardGroundNormal(0, 0, 1);// 旋转前的法向量
    groundEstimateNormal.normalize();
 
    // float angle = acos(groundEstimateNormal.dot(standardGroundNormal));// 向量间的夹角
    // Eigen::Vector3f p_rotate = groundEstimateNormal.cross(standardGroundNormal);// 两个向量组成平面的法向量

    float angle = acos(standardGroundNormal.dot(groundEstimateNormal));// 向量间的夹角
    Eigen::Vector3f p_rotate = standardGroundNormal.cross(groundEstimateNormal);// 两个向量组成平面的法向量
    p_rotate.normalize();
 
    Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle)) - p_rotate[2] * sin(angle);
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
 
    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
 
    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) +p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));
 
    return rotationMatrix;
}