#ifndef AREAPATH_H
#define AREAPATH_H

#include <math.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <boost/shared_ptr.hpp>

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

using namespace std;

constexpr double PI = 3.1415926;

//地图矩阵中各元素所赋予的数值
#define SIGN_NULL -1     //灰区
#define SIGN_EMPTY 0     //白区
#define SIGN_OBSTACLE 80 //膨胀障碍物区
#define SIGN_WALL 100    //黑区

#define DIRECTION_X 1 // x扫描方向
#define DIRECTION_Y 2 // y扫描方向

namespace area_path
{

    class AreaPath
    {
    public:
        AreaPath(nav_msgs::OccupancyGrid map);

        void AddAreaInPixels(unsigned int pointAx, unsigned int pointAy, unsigned int pointBx, unsigned int pointBy); //设置区域并计算路径（使用栅格点为单位）
        void AddAreaInMeters(double pointAxMeter, double pointAyMeter, double pointBxMeter, double pointByMeter);     //设置区域并计算路径（使用米为单位）

        bool CalculatePath();     //计算路径
        nav_msgs::Path GetPath(); //获取路径
        bool ClearPath();         //清除路径
        void PublishAreaPath();   //外部调用，发布矩形框内路径

    private:
        int ROW_SIZE;        //行间距
        int OBSTACLE_RADIUS; //障碍物膨胀半径，影响路径与墙距离
        int MAIN_DIRECTION;  //主方向，1为x主方向路径，2为y主方向路径
        double MIN_LENGTH;   //线段的最小长度，如果小于该值，线段将被去除

        ros::Publisher m_pathPub; //路径发布者

        nav_msgs::OccupancyGrid m_map; //静态地图
        unsigned int m_mapWidth;       //地图宽度
        unsigned int m_mapHeight;      //地图高度
        double m_resolution;           //地图的分辨率
        double m_mapOffsetX;           //地图原点origin偏置x
        double m_mapOffsetY;           //地图原点origin偏置y
        bool m_isCalculated;           //完成计算标记

        unsigned int m_pointAx, m_pointAy, m_pointBx, m_pointBy; // area的两顶点坐标

        std::vector<geometry_msgs::PoseStamped> m_poseStampedVector;       //原始位姿点列
        std::vector<geometry_msgs::PoseStamped> m_sortedPoseStampedVector; //排序后位姿点列
        long unsigned int m_sortedSize;
        nav_msgs::Path m_path; //路径

        void PrintMap();       //控制台预览地图，横轴x纵轴y
        void ObstacleExpand(); //地图障碍物膨胀
        void SortPoints();     //将点列排序

        void AppendPathInArea();                                                    //计算并加入自适应方向的路径
        std::vector<geometry_msgs::PoseStamped> GetPointsInArea(int MainDirection); //计算区域内的点列

        void AddPoint(unsigned int pointX, unsigned int pointY, int direction, std::vector<geometry_msgs::PoseStamped> &PoseStampedVector); //在点列中加入点
        nav_msgs::Path PoseStamped2Path(const std::vector<geometry_msgs::PoseStamped> &path); //将点列转换成路径
    };

} // namespace area_path

#endif // AREAPATH_H
