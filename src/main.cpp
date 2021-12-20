#include <iostream>
#include <unistd.h>
#include "utils/lidarMap_t.h"
#include "utils/typeConvert.h"
#include "paramFile/paramParse.h"
#include "lidarMapManage.h"
#include "visualization/visualize.h"
#include "groundSeg/ground_segmentation.h"
#include "objDetect/objDetect.h"
#include "gridMap/gridMapManage.h"
#include "groundPlaneCalibration/groundPlaneCali.h"
#include "lidarOdom/featureExtraction.h"
#include "lidarOdom/lidarOdometry.h"
#include "lidarOdom/pointCloudSeg.h"
using namespace std;

void LidarMapThreadFun(volatile bool &RunFlag, const InitParams& GlobalParams, LidarMap_t& lidarMap_th){
    /****** 启动线程读取雷达数据 ******/
    PointCloudQueue<vector<PointCloud_I>, double, long long> PCIPtr_Q16(5);// 声明存放点云的消息队列
    int Frequence_16 = 10;
    bool offLineFlag = false;
    string pcapAddr = GlobalParams.pcapAddr;
    if(pcapAddr != "") offLineFlag = true;
    /**************** 初始化16线雷达 **************/
    robosense::rslidar::ST_Param param_RS16;
    // 初始化16线雷达解码参数
    initRSLidar16Param(param_RS16);
    // 初始化16线雷达解码器
    robosense::rslidar::RSLidarDecoder<PointXYZITS> decoder_RS16(param_RS16);
    string device_ip16 = "192.168.3.200";// 32线ip
    //string device_ip16 = "192.168.1.216";
    string pcap_file_dir16 = pcapAddr;
    uint16_t msop_port16 = 6699,difop_port16 = 7788;// 32线端口号
    //uint16_t msop_port16 = 6616,difop_port16 = 7716;
    robosense::rslidar_input::Input InputObj_RS16(device_ip16,msop_port16,difop_port16,pcap_file_dir16);

    thread ListenRS16(listenRSLidar, ref(decoder_RS16), ref(InputObj_RS16), ref(PCIPtr_Q16), 32,
                      offLineFlag, Frequence_16, ref(RunFlag)); //16线ID号不连续
    /**************** 初始化32线雷达 ******************/

    /*** 声明变量 ***/
    double time_head_us16 = 0;// 当前帧头部时间
    long long time_tail_us16 = 0;// 当前帧尾部时间
    vector<PointCloud_I> PC_I_raw16_scanID(32);// 当前点云帧，按线束划分

    PointCloud_I::Ptr PC_I_raw_RS16(new PointCloud_I); // XYZI型原始点云信息
    PointCloud::Ptr PC_Raw_RS16(new PointCloud);       // XYZ型原始点云信息
    PointCloud_I::Ptr ground_cloud(new PointCloud_I), not_ground_cloud(new PointCloud_I);     // 分割的地面点云和障碍物点云
    PointCloud::Ptr ground_cloud_xyz(new PointCloud), not_ground_cloud_xyz(new PointCloud); // 用于显示效果

    PointCloud_I::Ptr targetCenterCloud(new PointCloud_I), targetCubeCloud(new PointCloud_I);// 目标检测中心区域点云和整体框选点云
    PointCloud_I::Ptr map_build_cloud(new PointCloud_I);
    PointCloud::Ptr map_base_plane(new PointCloud), map_base_plane_temp(new PointCloud);
    /**** 声明引导人员信息 *****/
    GuideObject guideObj, lastGuideObj;
    double last_time_stt_us = 0;// 上一帧时间戳

    /****** 显示工具初始化 ******/
    VisualTool myVisual;
    if(GlobalParams.visualize){
        myVisual.visualize(ground_cloud_xyz, not_ground_cloud_xyz);
    }
    while(RunFlag){
            /*********** 启动算法主体函数 ****************/
        // 获取原始点云信息
        PCIPtr_Q16.latestElem(&PC_I_raw16_scanID, &time_head_us16, &time_tail_us16);
        // 清除前一帧处理的点云信息
        PC_I_raw_RS16->clear();
        PC_Raw_RS16->clear();
        ground_cloud->clear();
        not_ground_cloud->clear();
        targetCenterCloud->clear();
        targetCubeCloud->clear();
        // // 测量雷达两帧之间的时间间隔
        // int time_interval_ms;
        // if(last_time_stt_us!=0){
        //     time_interval_ms = int((time_head_us16-last_time_stt_us)/1000);
        //     if(!time_interval_ms) {
        //         usleep(15*1000);
        //         continue;
        //     }
        //     //cout << "两帧时间间隔：　" << time_interval_ms << "ms" << endl;
        // }
        // last_time_stt_us = time_head_us16 ;
        // 拼接点云信息，获得点云对象
        for(auto &PC_I : PC_I_raw16_scanID){
            *PC_I_raw_RS16 += PC_I;
            PC_I.clear();
        }
        if(PC_I_raw_RS16->size() < 500){
            cout << "镜头遮挡，无法计算" << endl;
            continue;
        }
        // 1. 提取地面点
        GroundSegmentation segmenter(GlobalParams);
        std::vector<int> labels;// 标记数组，1是地面点，0是非地面点。
        XYZI2XYZ(PC_I_raw_RS16, PC_Raw_RS16);
        segmenter.segment(*PC_Raw_RS16, &labels);
        for (size_t i = 0; i < PC_I_raw_RS16->size(); ++i) {
            if (labels[i] == 1) ground_cloud->push_back(PC_I_raw_RS16->at(i));//1：标记为地面点
            else not_ground_cloud->push_back(PC_I_raw_RS16->at(i));
        }
        if(ground_cloud->size() < 500){
            cout<<"缺失地面点云，此帧处理结束"<<endl;
            continue;
        }
        // 在这个地方完成地面方程估计
        // 2. 地面方程估计，进行地平面矫正
        GroundPlaneCali  groundCali(GlobalParams);
        pcl::ModelCoefficients PlaneCoeff2Show;// 定义平面方程参数对象
        bool needGroundCali = false;
        XYZI2XYZ(ground_cloud, ground_cloud_xyz);
        groundCali.groundEquationEstimate(ground_cloud_xyz, PlaneCoeff2Show, needGroundCali);
        if(needGroundCali){
            Eigen::Matrix3f rotationMatrix = groundCali.getCaliRotateMatrix(PlaneCoeff2Show);
            //矫正完之后，地面平面为xoy平面，修改地面方程的参数
            PlaneCoeff2Show.values[0] = 0;
            PlaneCoeff2Show.values[1] = 0;
            PlaneCoeff2Show.values[2] = 1;
            PlaneCoeff2Show.values[3] = 0;
            //矫正所有点云，因为地面点云后续没有使用，可以不进行旋转
            convertPointCloud(not_ground_cloud, rotationMatrix);
        }
        XYZI2XYZ(not_ground_cloud, not_ground_cloud_xyz);
        // 3. 提取引导人员信息
        ObjDetect objDetector(GlobalParams);
        objDetector.getObjectCloud(not_ground_cloud, targetCenterCloud, targetCubeCloud);
        bool getTargetFlag = false;
        if(objDetector.getObjectCloudPosition(targetCenterCloud, guideObj, time_head_us16)){
            if(GlobalParams.visualize){
                PointT aboveGuidLogo(guideObj.position.x, guideObj.position.y, guideObj.position.z+0.5f);
                myVisual.addSphere(aboveGuidLogo, 0.25, 255, 0, 0, "target_sphere", 0);
            }
            // 计算速度（目标相对激光雷达坐标系的速度）
            objDetector.getObjectCloudVel(guideObj, lastGuideObj);
            // 删除引导人员周围点云，获取用于生成地图的点云
            objDetector.delObjectCubeCloud(guideObj, not_ground_cloud, map_build_cloud);
            getTargetFlag = true;
        }else{
            *map_build_cloud = *not_ground_cloud;
        }
        // 4. 生成栅格地图
        XYZI2XYZ(map_build_cloud, map_base_plane_temp);
        GridMapManage gridMapMag(GlobalParams);
        gridMapMag.groundCastFilter(map_base_plane_temp, PlaneCoeff2Show, map_base_plane);
        gridMapMag.planeCloud2Gridmap(map_base_plane);

        cv::Mat gridMat;// 用于显示栅格图像
        if(GlobalParams.visualize){
            gridMat = gridMapMag.gridMap2Mat();
            myVisual.imShowCVMat("grid", gridMat);
        }
        // 如果获取到了引导目标，就进行栅格地图信息的提取，以及目标在栅格上的定位
        if(getTargetFlag){
            LidarMap lidarmap;
            lidarmap.start_column =  gridMapMag.getLaserPosNumX();
            lidarmap.start_row = gridMapMag.getLaserPosNumY();
            gridMapMag.returnGridmapVector(lidarmap.map);
            gridMapMag.RealCoord2GridCoord(guideObj.position.x, guideObj.position.y,
                                            lidarmap.target_column, lidarmap.target_row);
            lidarMap_th.push(lidarmap); 
            // TODO:调用轨迹规划
            // 显示起点和目标点 
            // if(GlobalParams.visualize){
            //     myVisual.pointShow(0, lidarmap.start_column, lidarmap.start_row, GlobalParams.n_pixel_per_grid, gridMat,"grid");
            //     myVisual.pointShow(1, lidarmap.target_column, lidarmap.target_row, GlobalParams.n_pixel_per_grid, gridMat,"grid");
            // }
        }
        // 更新显示
        if(GlobalParams.visualize){
            myVisual.imShowCVMat("grid", gridMat);
            // myVisual.myWaitKey(0);这一行报错
            myVisual.myWaitKey(100);
            myVisual.updateVisual(ground_cloud_xyz, "ground_cloud");
            myVisual.updateVisual(not_ground_cloud_xyz, "obstacle_cloud");
            myVisual.spinOnce();
        }
    }  
}

void AutonomousNav(volatile bool &RunFlag, const InitParams& GlobalParams, LidarMap_t& lidarMap_th, Channel<OdometryOut>& odometry_out_channel,
                    Channel<ExtractionOut>& featureExtra_out_channel){
    /****** 启动线程读取雷达数据 ******/
    PointCloudQueue<vector<PointCloud_I>, double, long long> PCIPtr_Q16(5);// 声明存放点云的消息队列
    int Frequence_16 = 10;
    bool offLineFlag = false;
    string pcapAddr = GlobalParams.pcapAddr;
    if(pcapAddr != "") offLineFlag = true;
    /**************** 初始化16线雷达 **************/
    robosense::rslidar::ST_Param param_RS16;
    // 初始化16线雷达解码参数
    initRSLidar16Param(param_RS16);
    // 初始化16线雷达解码器
    robosense::rslidar::RSLidarDecoder<PointXYZITS> decoder_RS16(param_RS16);
    string device_ip16 = "192.168.3.200";// 32线ip
    //string device_ip16 = "192.168.1.216";
    string pcap_file_dir16 = pcapAddr;
    uint16_t msop_port16 = 6699,difop_port16 = 7788;// 32线端口号
    //uint16_t msop_port16 = 6616,difop_port16 = 7716;
    robosense::rslidar_input::Input InputObj_RS16(device_ip16,msop_port16,difop_port16,pcap_file_dir16);

    thread ListenRS16(listenRSLidar, ref(decoder_RS16), ref(InputObj_RS16), ref(PCIPtr_Q16), 32,
                      offLineFlag, Frequence_16, ref(RunFlag)); //16线ID号不连续

    /************* 自主导航线程 **************/
    // 声明算法对象
    PointCloudSeg PointCloudSeger(GlobalParams);
    FeatureExtraction FeatureExtracter(featureExtra_out_channel);
    // 里程计对象声明之后，计算线程就会开启，等待featureExtra_out_channel队列中有数据就会开始计算
    LidarOdometry LidarOdomer(featureExtra_out_channel, odometry_out_channel);

    double time_head_us16 = 0;// 当前帧头部时间
    long long time_tail_us16 = 0;// 当前帧尾部时间
    vector<PointCloud_I> PC_I_raw16_scanID(32);// 当前点云帧，按线束划分

    PointCloud_I::Ptr PC_I_raw_RS16(new PointCloud_I); // XYZI型原始点云信息
    PointCloud_I::Ptr ground_cloud(new PointCloud_I), not_ground_cloud(new PointCloud_I);     // 分割的地面点云和障碍物点云

    PointCloud_I::Ptr segmentedCloud(new PointCloud_I);
    PointCloud::Ptr ground_cloud_xyz(new PointCloud);
    PointCloud::Ptr map_base_plane(new PointCloud), map_base_plane_temp(new PointCloud);
    SegInfo segMsg;
    while(RunFlag){
        // 获取原始点云信息
        PCIPtr_Q16.latestElem(&PC_I_raw16_scanID, &time_head_us16, &time_tail_us16);
        // 清除前一帧处理的点云信息
        PC_I_raw_RS16->clear();
        ground_cloud->clear();
        not_ground_cloud->clear();
        for(auto &PC_I : PC_I_raw16_scanID){
            *PC_I_raw_RS16 += PC_I;
            PC_I.clear();
        }
        if(PC_I_raw_RS16->size() < 500){
            cout << "镜头遮挡，无法计算" << endl;
            continue;
        }
        // 数据预分割
        PointCloudSeger.cloudHandler(PC_I_raw_RS16);
        for (size_t i = 0; i < PC_I_raw_RS16->size(); ++i) {
            if (PointCloudSeger.groundLabel_[i] == 1) ground_cloud->push_back(PC_I_raw_RS16->at(i));//1：标记为地面点
            else not_ground_cloud->push_back(PC_I_raw_RS16->at(i));
        }
        if(ground_cloud->points.size() < 500){
            cout<<"缺失地面点云"<<endl;
            continue;
        }
        PointCloudSeger.publishSegInfo(segmentedCloud, segMsg);
        // 特征点提取
        FeatureExtracter.runFeatureAssociation(segmentedCloud, segMsg);
        FeatureExtracter.publishFeatureCloud();
        // 生成栅格地图
        // 1.1 地面方程估计，进行地平面矫正
        GroundPlaneCali  groundCali(GlobalParams);
        pcl::ModelCoefficients PlaneCoeff2Show;// 定义平面方程参数对象
        bool needGroundCali = false;
        XYZI2XYZ(ground_cloud, ground_cloud_xyz);
        groundCali.groundEquationEstimate(ground_cloud_xyz, PlaneCoeff2Show, needGroundCali);
        if(needGroundCali){
            Eigen::Matrix3f rotationMatrix = groundCali.getCaliRotateMatrix(PlaneCoeff2Show);
            //矫正完之后，地面平面为xoy平面，修改地面方程的参数
            PlaneCoeff2Show.values[0] = 0;
            PlaneCoeff2Show.values[1] = 0;
            PlaneCoeff2Show.values[2] = 1;
            PlaneCoeff2Show.values[3] = 0;
            //矫正所有点云，因为地面点云后续没有使用，可以不进行旋转
            convertPointCloud(not_ground_cloud, rotationMatrix);
        }
        // 1.2 生成栅格地图
        XYZI2XYZ(not_ground_cloud, map_base_plane_temp);
        GridMapManage gridMapMag(GlobalParams);
        gridMapMag.groundCastFilter(map_base_plane_temp, PlaneCoeff2Show, map_base_plane);
        gridMapMag.planeCloud2Gridmap(map_base_plane);
        LidarMap lidarmap;
        lidarmap.start_column =  gridMapMag.getLaserPosNumX();
        lidarmap.start_row = gridMapMag.getLaserPosNumY();
        gridMapMag.returnGridmapVector(lidarmap.map);
        lidarMap_th.push(lidarmap); 
    }
}

int main(){
    // 地图输出消息队列
    LidarMap_t lidarMap_th;
    // 里程计信息输出队列
	Channel<OdometryOut> odometry_out_channel;// true为阻塞传输
    // 分割聚簇队列
    Channel<ExtractionOut> featureExtra_out_channel;// true为阻塞传输
    volatile bool RunFlag = true;

    InitParams setParam;// 声明并初始化设定参数
    getParameter(setParam);
    if(setParam.pattern_select_switch == 0){
        thread LeaderThread(LidarMapThreadFun, ref(RunFlag), ref(setParam), ref(lidarMap_th));
        LeaderThread.join();
    }
    else if(setParam.pattern_select_switch == 1){
        thread NavigationThread(AutonomousNav, ref(RunFlag), ref(setParam), ref(lidarMap_th),
                                ref(odometry_out_channel), ref(featureExtra_out_channel));
        while(true){
            //取里程计值
            OdometryOut odomOut;
            odometry_out_channel.pop_uptodate(odomOut);
            for(int i=0; i<6; i++){
                cout<<odomOut.transformDataSum[i]<<" ";
            }
            cout<<endl;
        }
        NavigationThread.join();
    }

    cout << endl << "main exit in usually." << endl;
    return 0;
}