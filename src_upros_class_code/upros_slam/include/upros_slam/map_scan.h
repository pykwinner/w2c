#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include <vector>
#include <fstream>
#include <math.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

using namespace std;
// define point for obtain point set with bresenham
struct MapPoint
{
    int x, y;
    MapPoint()
    {
        x = 0;
        y = 0;
    }

    MapPoint(int x0, int y0)
    {
        x = x0;
        y = y0;
    }
};

// define MySlam class
class MySlam
{
public:
    MySlam(ros::NodeHandle &nh, double mapreso, double mposx, double mposy,
           double mposz, double morientx, double morienty, double morientz,
           double morientw, int mwidth, int mheight);
    ~MySlam();
    void mappub_init();
    void lasersub_init();
    void lasercallback(const sensor_msgs::LaserScanConstPtr &laserdata);
    void mapdata_init();
    vector<MapPoint> bresenham(int x0, int y0, int x1, int y1);

private:
    ros::NodeHandle nh_;
    ros::Subscriber lasersub_;
    ros::Publisher mappub_;
    tf::TransformListener tflistener_;

    nav_msgs::OccupancyGrid mapdata_;

    // define map reso, position:x,y,z,orientation:x,y,x,w
    double mapreso, mposx, mposy, mposz,
        morientx, morienty, morientz,
        morientw;

    // 地图宽度和高度
    int mwidth, mheight;
    vector<MapPoint> endpoints;
    MapPoint endpoint;
    vector<MapPoint> mappoints;

    tf::StampedTransform base2map;

    tf::Quaternion quat;

    double theta;

    tf::Vector3 trans_base2map;

    double tx, ty;

    int basex0, basey0;

    // scan beams end coordination in laser and map frame;
    double basex, basey;
    double mapx, mapy;

    // laser beams angle in laser frame
    double beamsAngle;
    int mapxn, mapyn;
    int laserNum;
    int nx, ny;
    int idx;

    // save data to file as log for problem check
    ofstream fopen;
    int scan_count;
    int scan_reso;
    boost::mutex map_mutex;
};
