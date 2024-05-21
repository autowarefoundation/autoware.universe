#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <vector>
#include <string>
#include <stdint.h>

using namespace std;

const float speed_proportion = 0.3;       //速度离散比例系数
//const float trans_para_forward = 0;     //将车辆定位转换到车头
const float trans_para_back = 0.5;     //将车辆定位转换到houlun 5 4
const float trans_para_park = 0;      //将车辆定位转换到车尾    2


//主车当前位置
struct CurrentPose
{
  double x;
  double y;
  double s;
  double d;
  double yaw;
};


//车道线信息,地图坐标系上
struct Point
{
    double x;
    double y;
    double s;
};
struct LaneLine
{
    vector<Point> points;
};




#endif // DATA_TYPE_H

