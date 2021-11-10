#ifndef GRID_H
#define GRID_H

#include "utils/common.h"

enum gridState{FREE, OCCU, UNKNOW};// 分别对应  0 1 2

class Grid{
public:
    Grid(){
        _state = UNKNOW;
        _visitNum = 0;//只有被判断为Free或者Occu才算访问
        _freeNum = 0;
        _occuNum = 0;
    };

    Grid(gridState state, unsigned int visitNum, unsigned int freeNum, unsigned int occuNum){
        _state = state;
        _visitNum = visitNum;
        _freeNum = freeNum;
        _occuNum = occuNum;
    };

    void addVisit(gridState visitState){
        if(visitState != UNKNOW) {
            ++_visitNum;
            if(visitState == FREE) ++_freeNum;
            else if(visitState == OCCU) ++_occuNum;
            else cerr << "gridMapManage.h addVisit: error visitState!" << endl;
        }
        else return;

        if(_visitNum == 1) _state = visitState;

        if(_visitNum > 2){
            if(_state == FREE && ((float)_occuNum) / _visitNum > 0.666) _state=OCCU;
            if(_state == OCCU && ((float)_freeNum) / _visitNum > 0.666) _state=FREE;
        }
    }

    gridState state(){return _state; }
    unsigned int visitNum() const{return _visitNum; }
    unsigned int freeNum() const{return _freeNum; }
    unsigned int occuNum() const{return _occuNum; }

private:
    gridState _state;
    unsigned int _visitNum;//只有被判断为Free或者Occu才算访问
    unsigned int _freeNum;
    unsigned int _occuNum;
};

#endif