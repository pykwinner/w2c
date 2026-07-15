#include "upros_slam/map_scan.h"

MySlam::MySlam(ros::NodeHandle &nh, double mapreso, double mposx, double mposy,
               double mposz, double morientx, double morienty, double morientz,
               double morientw, int mwidth, int mheight) : nh_(nh), mapreso(mapreso),
                                                           mposx(mposx), mposy(mposy), mposz(mposz), morientx(morientx),
                                                           morienty(morienty), morientz(morientz), morientw(morientw),
                                                           mwidth(mwidth), mheight(mheight)
{
    mapdata_init();
    mappub_init();
    lasersub_init();
}

MySlam::~MySlam()
{
}

// 激光雷达回调
void MySlam::lasercallback(const sensor_msgs::LaserScanConstPtr &laserdata)
{

    if (scan_count % scan_reso == 0)
    {
        try
        {
            tflistener_.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(3.0));
            tflistener_.lookupTransform("odom", "base_link", ros::Time(0), base2map);
        }
        catch (tf::TransformException &ex)
        {
            ROS_INFO("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        boost::mutex::scoped_lock map_lock(map_mutex);

        // 获取机器人坐标系在世界坐标系下姿态角
        quat = base2map.getRotation();
        theta = quat.getAngle();
        // 获取机器人坐标系在世界坐标系下位置
        trans_base2map = base2map.getOrigin();
        // 放到中间，防止数组溢出
        tx = trans_base2map.getX() + 10;
        ty = trans_base2map.getY() + 10;
        // 离散地图框架下机器人位置
        basex0 = int(tx / mapreso);
        basey0 = int(ty / mapreso);
        laserNum = laserdata->ranges.size();

        for (int i = 0; i < laserNum; i++)
        {
            beamsAngle = laserdata->angle_min + i * laserdata->angle_increment;
            // 激光点位于base坐标系下
            basex = laserdata->ranges[i] * cos(beamsAngle);
            basey = laserdata->ranges[i] * sin(beamsAngle);
            // 激光点转化到世界坐标系下
            mapx = basex * cos(theta) + basey * sin(theta) + tx;
            mapy = basey * cos(theta) - basex * sin(theta) + ty;
            // 离散到栅格
            nx = int(mapx / mapreso);
            ny = int(mapy / mapreso);
            mapxn = nx + 1;
            mapyn = ny + 1;
            endpoint.x = mapxn;
            endpoint.y = mapyn;
            endpoints.push_back(endpoint); // 添加到集合中
        }

        cout << "scan numbers are: " << endpoints.size() << endl;
        for (vector<MapPoint>::iterator iter = endpoints.begin(); iter != endpoints.end(); iter++)
        {
            // bresenham直线离散算法，中间的点全部可通行
            mappoints = MySlam::bresenham(basex0, basey0, (*iter).x, (*iter).y);

            cout << "bresenham point nums are:" << mappoints.size() << endl;

            for (vector<MapPoint>::iterator iter1 = mappoints.begin(); iter1 != mappoints.end(); iter1++)
            {
                idx = mwidth * (*iter1).y + (*iter1).x;
                cout << "idx is" << (*iter1).x << " " << (*iter1).y << endl;
                if (idx > 0)
                {
                    mapdata_.data[idx] = 0;
                }
            }
            mappoints.clear();
        }
        endpoints.clear();
        mappub_.publish(mapdata_);
    }
    scan_count++;
}

// 返回线上的全部点
vector<MapPoint> MySlam::bresenham(int x0, int y0, int x1, int y1)
{
    vector<MapPoint> pp;
    MapPoint p;
    int dx, dy, h, a, b, x, y, flag, t;
    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    if (x1 > x0)
        a = 1;
    else
        a = -1;
    if (y1 > y0)
        b = 1;
    else
        b = -1;
    x = x0;
    y = y0;
    if (dx >= dy)
    {
        flag = 0;
    }
    else
    {
        t = dx;
        dx = dy;
        dy = t;
        flag = 1;
    }
    h = 2 * dy - dx;
    for (int i = 1; i <= dx; ++i)
    {
        p.x = x, p.y = y;
        pp.push_back(p);
        if (h >= 0)
        {
            if (flag == 0)
                y = y + b;
            else
                x = x + a;
            h = h - 2 * dx;
        }
        if (flag == 0)
            x = x + a;
        else
            y = y + b;
        h = h + 2 * dy;
    }
    return pp;
}

// 初始化map发布publisher
void MySlam::mappub_init()
{
    mappub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 100);
}

// 订阅激光雷达信息
void MySlam::lasersub_init()
{
    lasersub_ = nh_.subscribe("scan", 1, &MySlam::lasercallback, this);
}

// 地图信息初始化
void MySlam::mapdata_init()
{
    scan_count = 0;
    scan_reso = 1;

    ros::Time currtime = ros::Time::now();
    mapdata_.header.stamp = currtime;

    mapdata_.header.frame_id = "map";

    mapdata_.info.resolution = mapreso;

    mapdata_.info.width = mwidth;
    mapdata_.info.height = mheight;

    mapdata_.info.origin.position.x = mposx;
    mapdata_.info.origin.position.y = mposy;
    mapdata_.info.origin.position.z = mposz;

    mapdata_.info.origin.orientation.x = morientx;
    mapdata_.info.origin.orientation.y = morienty;
    mapdata_.info.origin.orientation.z = morientz;
    mapdata_.info.origin.orientation.w = morientw;

    int dataszie = mwidth * mheight;
    mapdata_.data.resize(dataszie);

    for (int i = 0; i < dataszie; i++)
    {
        mapdata_.data[i] = -1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MySlam");

    ros::NodeHandle nh;

    // 地图参数
    double mapreso = 0.05, mposx = 0, mposy = 0, mposz = 0, morientx = 0, morienty = 0, morientz = 0, morientw = 1;

    int mwidth = 400, mheight = 400;

    MySlam myslam(nh, mapreso, mposx, mposy, mposz, morientx, morienty, morientz, morientw, mwidth, mheight);

    ros::spin();
    return 0;
}
