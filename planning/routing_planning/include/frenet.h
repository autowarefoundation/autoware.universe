#ifndef FRENET_H
#define FRENET_H

#include <vector>
#include <cmath>
#include <stdint.h>
#include <Eigen/Core>

using namespace std;

// For converting back and forth between radians and degrees.
double pi();
double deg2rad(double x);    //将度转为弧度
double rad2deg(double x);    //将弧度转为度


/**
 * @brief distance 求两点的欧式距离
 * @param x1  point_1 x
 * @param y1  point_1 y
 * @param x2  point_2 x
 * @param y2  point_2 y
 * @return  返回两点间距
 */
double distance(double x1, double y1, double x2, double y2);


/**
 * @brief getFrenet 笛卡尔坐标系 转 Frenet 坐标系
 * @param x       笛卡尔坐标系 x
 * @param y       笛卡尔坐标系 y
 * @param maps_x  笛卡尔坐标系路径点x
 * @param maps_y  笛卡尔坐标系路径点y
 * @param s_start 本路段起始s值
 * @return        返回Frenet坐标系下 s d
 */
vector<double> getFrenet2(const double x,const double y, const vector<double> &maps_x, const vector<double> &maps_y, const double s_start);

/**
 * @brief getFrenet 笛卡尔坐标系 转 Frenet 坐标系
 * @param x       笛卡尔坐标系 x
 * @param y       笛卡尔坐标系 y
 * @param maps_x  笛卡尔坐标系路径点x
 * @param maps_y  笛卡尔坐标系路径点y
 * @return        返回Frenet坐标系下 s d
 */
vector<double> getFrenet(const double x,const double y, const vector<double> &maps_x, const vector<double> &maps_y);


/**
 * @brief getXY   Frenet 坐标系 转 笛卡尔坐标系
 * @param s       Frenet 坐标系 s
 * @param d       Frenet 坐标系 d
 * @param maps_s  Frenet 坐标系下路径点 s
 * @param maps_x  笛卡尔坐标系路径点x
 * @param maps_y  笛卡尔坐标系路径点y
 * @return        返回笛卡尔坐标系下 x y
 */
vector<double> getXY(const double s, const double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y,const vector<double> &maps_x, const vector<double> &maps_y);

#endif // FRENET_H
