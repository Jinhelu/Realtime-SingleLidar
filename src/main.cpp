#include <iostream>
#include <unistd.h>
#include "utils/lidarMapQueue.h"
#include "utils/common.h"
#include "paramFile/paramParse.h"
#include "lidarMapManage.h"
using namespace std;

// 判断程序是否用于调试
//#define test

const string paramAddr = "../paramFile/parameter.txt";
const string startIconAddr = "../paramFile/icon/start_15.jpg";
const string endIconAddr = "../paramFile/icon/end_15.jpg";

 
void LidarMapThreadFun(volatile bool &RunFlag, LidarMapQueue& lidarMap_t, const string& pcapAddr){
#ifndef test
    InitParam setParam;// 声明并初始化设定参数
    getParameter(paramAddr, setParam);
    GroundSegmentationParams groundSegParams;
    getGroundSegParameter(paramAddr, groundSegParams);
#endif
    // 启动线程读取雷达数据
    PointCloudQueue<vector<PointCloud_I>, double, long long> PCIPtr_Q16(5);// 声明存放点云的消息队列
    int Frequence_16 = 10;
    bool offLineFlag = false;
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
    // 设置显示操作

    // 声明变量
    double time_first_us16;// 当前帧头部时间
    long long arriveTime16;// 当前帧尾部时间
    vector<PointCloud_I> PC_I_raw16_scanID(32);// 当前点云帧，按线束划分
    PointCloud_I::Ptr PC_I_raw_RS16(new PointCloud_I);
    double last_time_stt_us = 0;
    
    // 主线程逻辑
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
        PCIPtr_Q16.latestElem(&PC_I_raw16_scanID, &time_first_us16, &arriveTime16);
        PC_I_raw_RS16->clear();

        
        // // 测量雷达两帧之间的时间间隔
        // int time_interval_ms;
        // if(last_time_stt_us!=0){
        //     time_interval_ms = int((time_stt_us-last_time_stt_us)/1000);
        //     if(!time_interval_ms) {
        //         usleep(15*1000);
        //         continue;
        //     }
        //     //cout << "两帧时间间隔：　" << time_interval_ms << "ms" << endl;
        // }
        // last_time_stt_us = time_stt_us ;

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
        PointCloud::Ptr PC_Raw_RS16(new PointCloud);// XYZ型原始点云信息
        pcl::copyPointCloud(*PC_I_raw_RS16, *PC_Raw_RS16);// 将XYZI类型转换为XYZ类型
        segmenter.segment(*PC_Raw_RS16, &labels);
        PointCloud_I::Ptr ground_cloud(new PointCloud_I), obstacle_cloud(new PointCloud_I);
        for (size_t i = 0; i < PC_I_raw_RS16->size(); ++i) {
            if (labels[i] == 1) ground_cloud->push_back(PC_I_raw_RS16->at(i));//1：标记为地面点
            else obstacle_cloud->push_back(PC_I_raw_RS16->at(i));
        }
        // 2. 提取引导人员信息
        // 3. 生成栅格地图
    }      
}

int main(){
    LidarMapQueue lidarMap_th(100);

    volatile bool RunFlag = true;
    const string pcapAddr = "../trees.pcap";

    thread LidarThread(LidarMapThreadFun, ref(RunFlag), ref(lidarMap_th), ref(pcapAddr));
    // TODO:ref

    usleep(200*1000*1000);
    RunFlag = false;
    
    LidarThread.join();

    cout << endl << "main exit in usually." << endl;
    return 0;
}