#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

class MapPub
{
public:
    MapPub(ros::NodeHandle &nh, double map_reso,
           int map_width, int map_height,
           double posx, double posy, double posz,
           double orientx, double orienty,
           double orientz, double orientw);

    ~MapPub();

    void mappub_init();

    void mapdata_init();

private:
    ros::NodeHandle nh_;
    ros::Publisher mappub_;
    nav_msgs::OccupancyGrid mapdata_;
    nav_msgs::MapMetaData datainfo_;
    double map_reso;
    int map_width, map_height;
    double posx, posy, posz;
    double orientx, orienty, orientz, orientw;
};
