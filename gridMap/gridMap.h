#ifndef GRIDMAP_H
#define GRIDMAP_H

#include "grid.h"

class GridMap{
public:
    //初始化删格地图  输入：图像宽高 删格宽高   初始化各个删格状态为2（未知）
    GridMap(int WidthGridNum = 120,int HeightGridNum = 120);
    //以读文件的方式创建对象
    GridMap(const string& adress);

    //从激光雷达坐标转化为原点坐标(从0开始)，栅格地图原点坐标系为世界坐标系
    // 输入：栅格中相对雷达的坐标x、坐标y；返回：栅格中相对栅格原点的坐标x、坐标y
    void laserCoord2GridCoord (int& x_laser,int& y_laser,int& x_grid,int& y_grid) const;

    //某删格在删格原点下的坐标(从0开始)转化为在激光雷达下的坐标
    // 输入：栅格中相对栅格原点的坐标x、坐标y；返回：栅格中相对雷达的坐标x、坐标y
    void gridCoord2LaserCoord(int &GridCordX,int &GridCordY,int &LaserCordX,int &LaserCordY) const;

    //读取栅格坐标系中相对雷达的点云所在删格状态
    gridState getGridStateinLaserCoord(int x,int y);
    //读取某个特定删格的状态 从第0个删格开始
    gridState getGridState(int x,int y){ return _YCoord[y][x].state();}
    unsigned int getGridVisitNum(int x,int y){ return _YCoord[y][x].visitNum();}
    unsigned int getGridFreeNum(int x,int y){ return _YCoord[y][x].freeNum();}
    unsigned int getGridOccuNum(int x,int y){ return _YCoord[y][x].occuNum();}

    //读取删格地图的长和宽
    int getWidthNum(){ return _YCoord[0].size(); }
    int getHeightNum(){ return _YCoord.size(); }
    int getLaserNumX(){ return _laser_x; }
    int getLaserNumY(){ return _laser_y; }

    //删格规模扩容  在原有删格坐标系下的 极限坐标(可以为负数) 珊格数从0开始
    void enlargeGridmap(int x_grid, int y_grid);

    //修改某个特定删格的状态 从第0个删格开始  x坐标 y坐标 x,y可以为负数
    void ModifyGridState(int x, int y, gridState state);

    //以雷达为原点的参考系修改特定删格状态
    void modifyGridStateinLaserCoord(int x_laser, int y_laser, gridState state);

    //以向量形式返回删格
    vector<vector<int>> returnGridmapVector();

    //将当前栅格地图信息输出为../saved/grid.txt
    bool SaveGrid(string filename = "grid.txt");
    
private:
    int _laser_x;//名义上偏左
    int _laser_y;//名义上偏上
    deque<deque<Grid> >  _YCoord;    //标号是Y序列 ,纵坐标，行数
};

#endif