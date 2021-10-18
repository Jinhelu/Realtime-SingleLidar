#include <iostream>
#include <unistd.h>
#include "utils/lidarMapQueue.h"
#include "utils/common.h"
#include "paramFile/paramParse.h"
#include "lidarMapManage.h"
using namespace std;

// 判断程序是否用于调试
#define test

const string paramAddr = "../paramFile/parameter.txt";
const string startIconAddr = "../paramFile/icon/start_15.jpg";
const string endIconAddr = "../paramFile/icon/end_15.jpg";

 
void LidarMapThreadFun(volatile bool &RunFlag,LidarMapQueue& lidarMap_t,string pcapAddr){
#ifndef test
    InitParam setParam;// 声明并初始化设定参数
    getParameter(paramAddr, setParam);
#endif
    // 读取雷达数据
    PointCloudQueue<vector<PointCloud_I>,double,long long> PCIPtr_Q16(5);// 声明存放点云的消息队列
    recvDataRSLidar16(RunFlag, setParam, pcapAddr， PCIPtr_Q16);
    
    // 设置显示操作

    // 声明变量
    double time_first_us16;// 当前帧头部时间
    long long arriveTime16;// 当前帧尾部时间
    vector<PointCloud_I> PC_I_raw16_scanID(32);// 当前点云帧，按线束划分
    PointCloud_I::Ptr PC_I_raw_RS16(new PointCloud_I),PC_I_raw_RS(new PointCloud_I),
            PC_I_target(new PointCloud_I),PC_I_NotGuide(new PointCloud_I),//存储高反射率点群
            PC_I_Combine(new PointCloud_I),PC_I_NotGround(new PointCloud_I);
    PointCloud_C::Ptr PC_C_RsRaw(new PointCloud_C),PC_C_NoGuide(new PointCloud_C),PC_C_Rs16_Raw(new PointCloud_C),
                    PC_C_Combine(new PointCloud_C),PC_C_NotGround(new PointCloud_C),PC_C_WithGround(new PointCloud_C);
    PointCloud::Ptr PC_temp(new PointCloud),pointcloud_2d(new PointCloud);
    
    double last_time_stt_us = 0;
    
    // 主线程逻辑
    while(RunFlag){
        #ifdef test 
        // 动态获取参数便于调试
        InitParam setParam;// 声明并初始化设定参数
        getParameter(paramAddr, setParam);
        #endif
        /*********** 启动算法主体函数 ****************/
        // 获取原始点云信息
        PCIPtr_Q16.latestElem(&PC_I_raw16_scanID, &time_first_us16, &arriveTime16);
        PC_C_RsRaw->clear();
        pointcloud_2d->clear();
        PC_I_NoGuide->clear();
        PC_C_NoGuide->clear();
        PC_I_raw_RS32->clear();
        PC_C_Rs32_Raw->clear();
        PC_I_raw_RS->clear();
        PC_I_raw_RS16->clear();
        PC_C_Rs16_Raw->clear();

        PC_I_Combine->clear();
        PC_C_Combine->clear();

        PC_I_NotGround->clear();
        PC_C_NotGround->clear();

        PC_C_WithGround->clear();
        
        // 测量雷达两帧之间的时间间隔
        int time_interval_ms;
        if(last_time_stt_us!=0){
            time_interval_ms = int((time_stt_us-last_time_stt_us)/1000);
            if(!time_interval_ms) {
                usleep(15*1000);
                continue;
            }
            //cout << "两帧时间间隔：　" << time_interval_ms << "ms" << endl;
        }
        last_time_stt_us = time_stt_us ;

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

        // 2. 提取引导人员信息

        // 3. 生成栅格地图
    }      
}

int main(){
    LidarMapQueue lidarMap_th(100);

    volatile bool RunFlag = true;
    const string pcapAdress = "../trees.pcap";

    thread LidarThread(LidarMapThreadFun,ref(RunFlag),ref(lidarMap_th),pcapAdress);
    // TODO:ref

    usleep(200*1000*1000);
    RunFlag = false;
    
    LidarThread.join();

    cout << endl << "main exit in usually." << endl;
    return 0;
}