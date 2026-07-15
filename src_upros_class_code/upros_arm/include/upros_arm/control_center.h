#ifndef CONTROL_CENTER_H
#define CONTROL_CENTER_H

#include "upros_arm_driver.h"

#include <math.h>

#include "upros_message/ArmPosition.h"
#include "std_srvs/Empty.h"
#include <algorithm>

using namespace std;

class ControlCenter
{

public:
    ControlCenter(ros::NodeHandle &nh) : nn(nh) {}
    ~ControlCenter() {}
    void initROSModule();

private:
    ros::NodeHandle nn;
    ros::ServiceServer arm_pos_open_claw_service;
    ros::ServiceServer arm_pos_close_claw_service;
    ros::ServiceServer grab_service;
    ros::ServiceServer release_service;
    ros::ServiceServer zero_service;

    UPROS_ARM upros_arm;

    bool arm_position_open_claw_callback(upros_message::ArmPosition::Request &req, upros_message::ArmPosition::Response &resp);
    bool arm_position_close_claw_callback(upros_message::ArmPosition::Request &req, upros_message::ArmPosition::Response &resp);

    bool grab_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool loose_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool zero_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
};

#endif
