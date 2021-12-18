#include "gridMap.h"

GridMap::GridMap(int WidthGridNum, int HeightGridNum){
    if (((WidthGridNum/2)*2)!=WidthGridNum || ((HeightGridNum/2)*2)!=HeightGridNum){
        cerr<<"输入删格必须为偶数！重新定义删格！"<<endl;
    }else{
        Grid temp;
        deque<Grid> XCoordState (WidthGridNum, temp);
        deque<deque<Grid> > YCoord(HeightGridNum, XCoordState);
        _YCoord = YCoord;
        _laser_x = WidthGridNum/4 - 1;//实际上不属于任何一个栅格，在两格中间
        _laser_y = HeightGridNum/2 - 1;//实际上不属于任何一个栅格，在两格中间
    }
}

//以读离线文件的方式创建对象
GridMap::GridMap(const string& adress){
    ifstream infile(adress);
    if (!infile)  { cerr<<"读取文件失败!"<<endl; }
    else{
        string temp_str;
        //读取激光雷达数据
        infile >> _laser_x >> _laser_y;
        getline(infile,temp_str);
        //读取文件到二维deque
        while(getline(infile,temp_str)){
            //行内的操作
            deque<Grid> test_line;
            stringstream temp_ss(temp_str);
            gridState state;
            unsigned int stateNum;
            unsigned int visitNum;
            unsigned int freeNum;
            unsigned int occuNum;
            while(temp_ss >> stateNum >> visitNum >> freeNum >> occuNum) {
                if(stateNum==0) state=FREE;
                else if(stateNum==1) state = OCCU;
                else state = UNKNOW;

                Grid temp(state,visitNum,freeNum,occuNum);
                test_line.push_back(temp);
            }
            _YCoord.push_back(test_line);
        }//整个文件读取完成
        //初始化private成员
        cout<<"文件读取完成!"<<" 行数： "<<this->getHeightNum()<<"列数： "<<this->getWidthNum()
            << endl<< "激光雷达原点： x："<< _laser_x  << "   y:" << _laser_y <<endl;
    }
}

//从激光雷达坐标转化为原点坐标(从0开始)，栅格地图原点坐标系为世界坐标系
// 输入：栅格中相对雷达的坐标x、坐标y；返回：栅格中相对栅格原点的坐标x、坐标y
void GridMap::laserCoord2GridCoord (int& x_laser,int& y_laser,int& x_grid,int& y_grid) const {
    if(x_laser >= 0) x_grid = _laser_x + x_laser;
    else x_grid = _laser_x + x_laser+1;

    if(y_laser > 0) y_grid = _laser_y - y_laser+1;
    else y_grid =_laser_y - y_laser;
}

//某删格在删格原点下的坐标(从0开始)转化为在激光雷达下的坐标
// 输入：栅格中相对栅格原点的坐标x、坐标y；返回：栅格中相对雷达的坐标x、坐标y
void GridMap::gridCoord2LaserCoord(int& x_grid, int& y_grid, int& x_laser, int& y_laser) const {
    if(x_grid >= _laser_x) x_laser = x_grid - _laser_x;
    else x_laser = x_grid - _laser_x -1;

    if( y_grid > _laser_y) y_laser =_laser_y - y_grid + 1;
    else y_laser = _laser_y- y_grid;
}

//读取栅格坐标系中相对雷达的点云所在删格状态
gridState GridMap::getGridStateinLaserCoord(int x_laser, int y_laser){
    int x_grid,y_grid;
    this->laserCoord2GridCoord(x_laser, y_laser, x_grid, y_grid);
    return _YCoord[y_grid][x_grid].state();
}

//删格规模扩容  在原有删格坐标系下的 极限坐标(可以为负数) 珊格数从0开始
void GridMap::enlargeGridmap(int x_grid, int y_grid){
    Grid temp;
    //判断X是否超出范围，超出范围就扩容
    if(x_grid >= (int)_YCoord[0].size()){
        int deltaX = x_grid - _YCoord[0].size() + 1;
        //x方向补齐
        while(deltaX){
            //每行都增加一个删格
            int col_flag = _YCoord.size();// 取队列的数目
            while(col_flag){_YCoord[col_flag-1].push_back(temp);  col_flag--;}
            deltaX--;
        }
        //cout << "x正方向扩容" << endl;
    }else if(x_grid < 0){
        int deltaX = (-x_grid);
        while(deltaX){
            //cout << "x负方向扩容" << deltaX <<endl;
            int col_flag = _YCoord.size();
            while(col_flag){_YCoord[col_flag-1].push_front(temp);  col_flag--; }
            deltaX--;
        }
        _laser_x += (-x_grid);
        //cout << "x负方向扩容" << endl;
    }

    //判断Y是否超出范围，超出范围就扩容
    if(y_grid >= (int)_YCoord.size()){
        deque<Grid> LineTemp(_YCoord[0].size(),temp);
        int deltaY = y_grid - _YCoord.size() +1;
        //Y方向补齐
        while(deltaY){_YCoord.push_back(LineTemp);  deltaY--;}
        //cout << "y正方向扩容" << endl;
    }else if(y_grid < 0){
        deque<Grid> LineTemp(_YCoord[0].size(),temp);
        int deltaY = (-y_grid);
        while(deltaY){_YCoord.push_front(LineTemp);  deltaY--;}
        _laser_y += (-y_grid);
        //cout << "y负方向扩容" << endl;
    }
}

//修改某个特定删格的状态 从第0个删格开始  x坐标 y坐标 x,y可以为负数
void GridMap::ModifyGridState(int x_grid, int y_grid, gridState state){
    if (state != FREE && state != OCCU && state != UNKNOW){
        cerr << "input _state is illegal!" << endl;
    }
    else{
        enlargeGridmap(x_grid, y_grid);
        if(x_grid >= 0 && y_grid >= 0) _YCoord[y_grid][x_grid].addVisit(state);
        else{
            if(x_grid < 0) x_grid = 0;
            if(y_grid < 0) y_grid = 0;
            _YCoord[y_grid][x_grid].addVisit(state);
        }
    }
}

//以雷达为原点的参考系修改特定删格状态
void GridMap::modifyGridStateinLaserCoord(int x_laser, int y_laser, gridState state){
    int x_grid,y_grid;
    this->laserCoord2GridCoord(x_laser, y_laser, x_grid, y_grid);// 雷达坐标系转换到栅格坐标系
    this->enlargeGridmap(x_grid, y_grid);//此时雷达在栅格图中的坐标_laser_x和_laser_y已经改变
    this->laserCoord2GridCoord(x_laser, y_laser, x_grid, y_grid);//一定是合法的 x_grid, y_grid
    this->ModifyGridState(x_grid, y_grid, state);
}

//以向量形式返回删格
vector<vector<int>> GridMap::returnGridmapVector(){
    vector<int> a;
    vector<vector<int> > YCoord_Vector(this->getHeightNum(), a);
    for(int j=0; j<this->getHeightNum(); j++){
        for(int i=0; i<this->getWidthNum(); i++){
            int oldState = 2;
            if(this->_YCoord[j][i].state() == FREE) oldState=0;
            else if(this->_YCoord[j][i].state() == OCCU) oldState=1;
            else oldState = 2;
            YCoord_Vector[j].push_back(oldState);
        }
    }
    return YCoord_Vector;
}

//将当前栅格地图信息输出为../saved/grid.txt
bool GridMap::SaveGrid(string filename){
    //创建文件对象
    ofstream outfile("../saved/" + filename, ofstream::out);
    if(!outfile){cerr << " 文件打开失败 "<<endl;  return false;}
    //写删格中的雷达位置
    outfile << _laser_x << " " << _laser_y << endl;
    for(int j=0;j<_YCoord.size();j++){
        //对每一行y
        for(int i=0;i<_YCoord[0].size();i++){
            //对每行的每个删格x
            outfile << getGridState(i,j) << " "<< getGridVisitNum(i,j) << " "
                    << getGridFreeNum(i,j) << " " << getGridOccuNum(i,j) << "   ";
        }
        outfile << endl;//换行即新的栅格行
    }
    return true;
}