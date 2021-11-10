#include <iostream>
#include <unistd.h>
#include "utils/lidarMapQueue.h"
#include "utils/common.h"
#include "utils/typeConvert.h"
#include "paramFile/paramParse.h"
#include "lidarMapManage.h"
#include "visualization/visualize.h"
#include "groundSeg/ground_segmentation.h"
#include "objDetect/objDetect.h"
#include "gridMap/gridMapManage.h"
using namespace std;

// 判断程序是否用于调试
//#define test
 
void LidarMapThreadFun(volatile bool &RunFlag, LidarMapQueue& lidarMap_t){
#ifndef test
    InitParam setParam;// 声明并初始化设定参数
    getParameter(setParam);
    GroundSegmentationParams groundSegParams;// 地面分割算法参数
    getGroundSegParameter(groundSegParams);
    ObjDetectParams objectDetectParams;// 目标识别参数
    getObjDetectParameter(objectDetectParams);
    GridMapParams gridMapParams;// 栅格地图参数
    getGridMapParameter(gridMapParams);

#endif
    /****** 启动线程读取雷达数据 ******/
    PointCloudQueue<vector<PointCloud_I>, double, long long> PCIPtr_Q16(5);// 声明存放点云的消息队列
    int Frequence_16 = 10;
    bool offLineFlag = false;
    string pcapAddr = setParam.pcapAddr;
    if(pcapAddr != "") offLineFlag = true;
    robosense::rslidar::ST_Param param_RS16;
    // 初始化16线雷达解码参数
    initRSLidar16Param(param_RS16, setParam);
    // 初始化16线雷达解码器
    robosense::rslidar::RSLidarDecoder<PointXYZITS> decoder_RS16(param_RS16);
    string device_ip16 = "192.168.1.216";
    string pcap_file_dir16 = pcapAddr;
    uint16_t msop_port16 = 6616,difop_port16 = 7716;
    robosense::rslidar_input::Input InputObj_RS16(device_ip16,msop_port16,difop_port16,pcap_file_dir16);

    thread ListenRS16(listenRSLidar, ref(decoder_RS16), ref(InputObj_RS16), ref(PCIPtr_Q16), 32,
                      offLineFlag, Frequence_16, ref(RunFlag)); //16线ID号不连续
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
    if(setParam.visualize){
        myVisual.visualize(ground_cloud_xyz, not_ground_cloud_xyz);
    }
    
    /********** 主线程逻辑 ****************/
    while(RunFlag){
        #ifdef test 
        // 动态获取参数便于调试
        InitParam setParam;// 声明并初始化人员跟随参数
        getParameter(paramAddr, setParam);
        GroundSegmentationParams groundSegParams;// 声明并初始化地面分割参数 
        getGroundSegParameter(paramAddr, groundSegParams);
        #endif
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
        for(auto &PC_I:PC_I_raw16_scanID){
            *PC_I_raw_RS16 += PC_I;
            PC_I.clear();
        }
        if(PC_I_raw_RS16->size()<500){
            cout << "镜头遮挡，无法计算" << endl;
            continue;
        }

        // 1. 提取地面点
        GroundSegmentation segmenter(groundSegParams);
        std::vector<int> labels;// 标记数组，1是地面点，0是非地面点。
        pcl::copyPointCloud(*PC_I_raw_RS16, *PC_Raw_RS16);// 将XYZI类型转换为XYZ类型
        segmenter.segment(*PC_Raw_RS16, &labels);
        for (size_t i = 0; i < PC_I_raw_RS16->size(); ++i) {
            if (labels[i] == 1) ground_cloud->push_back(PC_I_raw_RS16->at(i));//1：标记为地面点
            else not_ground_cloud->push_back(PC_I_raw_RS16->at(i));
        }
        if(ground_cloud->size()<100){
            cout<<"缺失地面点云，此帧处理结束"<<endl;
            continue;
        }
        XYZI2XYZ(ground_cloud, ground_cloud_xyz);
        XYZI2XYZ(not_ground_cloud, not_ground_cloud_xyz);
        // 2. 提取引导人员信息
        ObjDetect objDetector(objectDetectParams);
        objDetector.getObjectCloud(not_ground_cloud, targetCenterCloud, targetCubeCloud);
        bool getTargetFlag = false;
        if(objDetector.getObjectCloudPosition(targetCenterCloud, guideObj, time_head_us16)){
            if(setParam.visualize){
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
        // 3. 生成栅格地图
        pcl::ModelCoefficients PlaneCoeff2Show;// 定义平面方程参数对象
        
        XYZI2XYZ(map_build_cloud, map_base_plane_temp);
        GridMapManage gridMapMag(gridMapParams);
        gridMapMag.groundEquationEstimate(ground_cloud_xyz, PlaneCoeff2Show);
        gridMapMag.groundCastFilter(map_base_plane_temp, PlaneCoeff2Show, map_base_plane);
        gridMapMag.planeCloud2Gridmap(map_base_plane);

        cv::Mat gridMat;// 用于显示栅格图像
        if(setParam.visualize){
            gridMat = gridMapMag.gridMap2Mat();
            myVisual.imShowCVMat("grid", gridMat);
        }
        // 如果获取到了引导目标，就进行栅格地图信息的提取，以及目标在栅格上的定位
        if(getTargetFlag){
            LidarMap lidarmap;
            lidarmap.start_column =  gridMapMag.getLaserPosNumX();
            lidarmap.start_row = gridMapMag.getLaserPosNumY();
            // 下一行有bug
            gridMapMag.returnGridmapVector(lidarmap.map);
            gridMapMag.RealCoord2GridCoord(guideObj.position.x, guideObj.position.y,
                                            lidarmap.target_column, lidarmap.target_row);
            lidarMap_t.push(lidarmap); 
            // TODO:调用轨迹规划
            // 显示起点和目标点 
            if(setParam.visualize){
                myVisual.pointShow(0, lidarmap.start_column, lidarmap.start_row, gridMapParams.n_pixel_per_grid, gridMat,"grid");
                myVisual.pointShow(0, lidarmap.start_column, lidarmap.start_row, gridMapParams.n_pixel_per_grid, gridMat,"grid");
            }
        }
        // 更新显示
        if(setParam.visualize){
            myVisual.imShowCVMat("grid", gridMat);
            myVisual.myWaitKey(0);
            myVisual.updateVisual(ground_cloud_xyz, "ground_cloud");
            myVisual.updateVisual(not_ground_cloud_xyz, "obstacle_cloud");
            myVisual.spinOnce();
        }
    }
    ListenRS16.join();     
}

int main(){
    LidarMapQueue lidarMap_th(100);

    volatile bool RunFlag = true;

    thread LidarThread(LidarMapThreadFun, ref(RunFlag), ref(lidarMap_th));
    // TODO:ref

    usleep(200*1000*1000);
    RunFlag = false;
    
    LidarThread.join();

    cout << endl << "main exit in usually." << endl;
    return 0;
}