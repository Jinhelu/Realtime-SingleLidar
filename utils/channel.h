#ifndef CHANNEL_H
#define CHANNEL_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include "common.h"
using namespace std;

// OdometryOut里程计线程的输出信息格式,transformDataSum={rx, ry, rz, tx, ty, tz}, 角度制和米
struct OdometryOut{
    std::vector<float> transformDataSum;
    OdometryOut(){
        transformDataSum.resize(6, 0.0);
    }
};

// ExtractionOut 特征提取线程输出信息，里程计计算线程的输入信息
struct ExtractionOut{
    pcl::PointCloud<PointT_I> cornerPointsSharp;
    pcl::PointCloud<PointT_I> cornerPointsLessSharp;
    pcl::PointCloud<PointT_I> surfPointsFlat;
    pcl::PointCloud<PointT_I> surfPointsLessFlat;
    
    ExtractionOut& operator=(const ExtractionOut& s){
        this->cornerPointsSharp = s.cornerPointsSharp;
        this->cornerPointsLessSharp = s.cornerPointsLessSharp;
        this->surfPointsFlat = s.surfPointsFlat;
        this->surfPointsLessFlat = s.surfPointsLessFlat;
    }
};

template <typename T>
class Channel{
public:
    Channel(){
        _updateFlag = false;
        _container.resize(1);
    }

    void push(T &newElem){
        lock_guard<mutex> lk(_mut);
        _container[0] = newElem;
        _updateFlag = true;
        // Wake up the waiting thread.
        _cv.notify_one();
    }

    bool pop_uptodate(T &receiver){
        std::unique_lock<std::mutex> lk(_mut);
        // If the queue is empty, wait for data.
        _cv.wait(lk, [this]
        { return (_updateFlag); });
        receiver = _container[0];
        _updateFlag = false;
        return true;
    }

    bool IfUpdate(){
        lock_guard<mutex> lk(_mut);
        return _updateFlag;
    }

    void reset(){
        lock_guard<mutex> lk(_mut);
        _updateFlag = false;
    }
private:
    vector<T> _container;
    bool _updateFlag;
    mutex _mut;
    condition_variable _cv;
};


#endif