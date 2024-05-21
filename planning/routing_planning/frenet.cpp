#include "frenet.h"

// For converting back and forth between radians and degrees.
double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }  //将度转为弧度
double rad2deg(double x) { return x * 180 / pi(); }  //将弧度转为度


//->求两点的欧式距离

double distance(double x1, double y1, double x2, double y2){
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

//->向量夹角方向判断（逆时针为正，顺时针为负）,vector1指向vector2
int cross(Eigen:: Vector2f &vector1,Eigen:: Vector2f &vector2){
  return  (vector1[0] * vector2[1] - vector2[0] * vector1[1]) > 0 ? 1 : -1;
}

//->该方法存在重叠区和盲区问题,解决方法有二:
//->1. 尽可能的使参考路径平滑.
//->2. 使用纯跟踪算法中的方法处理相关路径段的定位.
int NextWaypoint(double x, double y,const vector<double> &maps_x, const vector<double> &maps_y){

    //-> 1.计算距离最近的点,遍历逼近的方式求解最小值
    double closestLen = 100000;
    int closestWaypoint = 0;
    int map_size = maps_x.size();
    for(uint32_t i = 0; i < map_size; i++){
      double map_x = maps_x[i];
      double map_y = maps_y[i];
      double dist = distance(x,y,map_x,map_y);
      if(dist < closestLen){
        closestLen = dist;
        closestWaypoint = i;
      }
    }
    //-> 2. 判断目标点所处的路径段,使用下一个路径点的记号表示,从1计数 第 i 段路径
    Eigen:: Vector2f line_1;      //->最近点与其最近的路径段组成路径段向量
    Eigen:: Vector2f point2pose;  //->最近点指向目标点的向量
    if(closestWaypoint != 0){
      line_1[0] = maps_x[closestWaypoint] - maps_x[closestWaypoint -1];
      line_1[1] = maps_y[closestWaypoint] - maps_y[closestWaypoint -1];
      point2pose[0] = x - maps_x[closestWaypoint];
      point2pose[1] = y - maps_y[closestWaypoint];

      //->计算两个向量的角度,如果是锐角(即 cos 值为正) 则下一个序号为路径段编号
      float acos = line_1.dot(point2pose) / line_1.norm()*point2pose.norm();
      closestWaypoint = acos >= 0 ? closestWaypoint + 1 : closestWaypoint; //acos >= 0将closestWaypoint + 1赋值给closestWaypoint
    }else{
      line_1[0] = maps_x[closestWaypoint+1] - maps_x[closestWaypoint];
      line_1[1] = maps_y[closestWaypoint+1] - maps_y[closestWaypoint];
      point2pose[0] = x - maps_x[closestWaypoint];
      point2pose[1] = y - maps_y[closestWaypoint];

      //->计算两个向量的角度,如果是锐角(即 cos 值为正) 则下一个序号为路径段编号
      float acos = line_1.dot(point2pose) / line_1.norm()*point2pose.norm();
      closestWaypoint = acos >= 0 ? closestWaypoint + 1 : closestWaypoint;
    }
    return (closestWaypoint >= map_size) ? (map_size-1) : closestWaypoint;//（closestWaypoint >= map_size）时将map_size-1赋值给closestWaypoint
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet2(const double x,const double y, const vector<double> &maps_x, const vector<double> &maps_y, const double s_start){
    int next_wp = NextWaypoint(x,y,maps_x,maps_y);
    Eigen:: Vector2f line_1;
    Eigen:: Vector2f Q;

    if(next_wp == 0) return{s_start,0};

    // if(next_wp == maps_x.size()){
    //   double frenet_s = 0;
    //   for(int i = 0; i < next_wp; i++){
    //     frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    //   }
    //   return {frenet_s+s_start,0};
    // }

    line_1[0] = maps_x[next_wp] - maps_x[next_wp -1];
    line_1[1] = maps_y[next_wp] - maps_y[next_wp -1];

    Q[0] = x - maps_x[next_wp -1];
    Q[1] = y - maps_y[next_wp -1];

    //->
    double proj_norm = (Q[0]*line_1[0]+Q[1]*line_1[1])/(line_1[0]*line_1[0]+line_1[1]*line_1[1]);
    double proj_x = proj_norm*line_1[0];
    double proj_y = proj_norm*line_1[1];
    double frenet_d = cross(Q,line_1) * distance(Q[0],Q[1],proj_x,proj_y);

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < next_wp - 1; i++){
      frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }
    frenet_s = frenet_s + distance(0,0,proj_x,proj_y);
    return {frenet_s+s_start,frenet_d};
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(const double x,const double y, const vector<double> &maps_x, const vector<double> &maps_y){
    int next_wp = NextWaypoint(x,y,maps_x,maps_y);
    Eigen:: Vector2f line_1;
    Eigen:: Vector2f Q;

    line_1[0] = maps_x[next_wp] - maps_x[next_wp -1];
    line_1[1] = maps_y[next_wp] - maps_y[next_wp -1];

    Q[0] = x - maps_x[next_wp -1];
    Q[1] = y - maps_y[next_wp -1];

    //->
    double proj_norm = (Q[0]*line_1[0]+Q[1]*line_1[1])/(line_1[0]*line_1[0]+line_1[1]*line_1[1]);
    double proj_x = proj_norm*line_1[0];
    double proj_y = proj_norm*line_1[1];
    double frenet_d = cross(Q,line_1) * distance(Q[0],Q[1],proj_x,proj_y);

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < next_wp - 1; i++){
      frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }
    frenet_s = frenet_s + distance(0,0,proj_x,proj_y);
    return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(const double s, const double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y){
    uint32_t prev_wp = 0;

    for (prev_wp = 0; prev_wp < maps_s.size() - 1;prev_wp++){
       if(s >= maps_s[prev_wp] && s < maps_s[prev_wp +1]) break;
     }

    double heading = atan2((maps_y[prev_wp +1]- maps_y[prev_wp]),(maps_x[prev_wp +1] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s*cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s*sin(heading);

    double perp_heading = heading- M_PI_2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}
