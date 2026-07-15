#include "upros_navigation/AreaPath.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "area_path_node");
    ros::NodeHandle global_nh;

    // 从服务获取地图
    ros::ServiceClient mapClient = global_nh.serviceClient<nav_msgs::GetMap>("static_map"); // server名称
    nav_msgs::GetMap mapSrv;
    ros::Rate r1(1);
    while (!mapClient.call(mapSrv))
    {
        ROS_ERROR("area_path_node Failed get static_map!!");
        r1.sleep();
    }
    ROS_INFO("area_path_node Successfully get static_map!!");
    area_path::AreaPath areaPath(mapSrv.response.map);

    // 设置清扫区域
    ros::NodeHandle private_nh("~/clean_area");
    std::vector<double> area_x_meter_list;
    std::vector<double> area_y_meter_list;
    std::vector<int> area_x_pixel_list;
    std::vector<int> area_y_pixel_list;
    private_nh.getParam("area_x_meter", area_x_meter_list);
    private_nh.getParam("area_y_meter", area_y_meter_list);
    private_nh.getParam("area_x_pixel", area_x_pixel_list);
    private_nh.getParam("area_y_pixel", area_y_pixel_list);

    if (area_x_meter_list.size() == area_y_meter_list.size() && area_x_meter_list.size() % 2 == 0)
    {
        for (size_t i = 0; i < area_x_meter_list.size() / 2; i++)
        {
            areaPath.AddAreaInMeters(area_x_meter_list[i * 2], area_y_meter_list[i * 2], area_x_meter_list[i * 2 + 1], area_y_meter_list[i * 2 + 1]);
        }
    }
    else
    {
        ROS_ERROR("area_meter's x & y can not match!");
    }

    if (area_x_pixel_list.size() == area_y_pixel_list.size() && area_x_pixel_list.size() % 2 == 0)
    {
        bool negativeFlag = false;
        for (size_t i = 0; i < area_x_pixel_list.size(); i++)
        {
            if (area_x_pixel_list[i] < 0 || area_y_pixel_list[i] < 0)
            {
                ROS_ERROR("area_pixel can not be negative!");
                negativeFlag = true;
                break;
            }
        }
        if (!negativeFlag)
        {
            for (size_t i = 0; i < area_x_pixel_list.size() / 2; i++)
            {
                areaPath.AddAreaInPixels((unsigned int)area_x_pixel_list[i * 2], (unsigned int)area_y_pixel_list[i * 2], (unsigned int)area_x_pixel_list[i * 2 + 1], (unsigned int)area_y_pixel_list[i * 2 + 1]);
            }
        }
    }
    else
    {
        ROS_ERROR("area_pixel's x & y can not match!");
    }

    areaPath.CalculatePath();

    // 计算完毕后发布路径
    ros::Rate r2(1);
    while (ros::ok())
    {
        areaPath.PublishAreaPath();
        ros::spinOnce();
        r2.sleep();
    }

    ros::shutdown();
    return 0;
}
