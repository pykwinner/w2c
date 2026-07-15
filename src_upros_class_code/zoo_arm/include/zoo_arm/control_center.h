#ifndef CONTROL_CENTER_H
#define CONTROL_CENTER_H

#include "mini2_arm.h"

#include <math.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include "arm_inverse.h"
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

    ros::ServiceServer armPos_service;
    ros::ServiceServer grab_service;
    ros::ServiceServer release_service;
    ros::ServiceServer zero_service;

    Mini2_ARM arm;
    ArmInverse arm_inverse;

    bool arm_position_callback(upros_message::ArmPosition::Request &req, upros_message::ArmPosition::Response &resp);
    bool grab_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool loose_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool zero_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

};

#endif
