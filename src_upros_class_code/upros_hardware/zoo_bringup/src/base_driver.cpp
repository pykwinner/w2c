#include "base_driver.h"
#include "data_holder.h"

#include <std_msgs/Float32MultiArray.h>
#include "serial_transport.h"
#include "simple_dataframe_master.h"
#include <boost/assign/list_of.hpp>

BaseDriver *BaseDriver::instance = NULL;

BaseDriver::BaseDriver() : pn("~"), bdg(pn)
{
    // 初始化配置
    bdg.init();

    trans = boost::make_shared<Serial_transport>(bdg.port, bdg.buadrate);
    frame = boost::make_shared<Simple_dataframe>(trans.get());

    ROS_INFO("BaseDriver startup");
    if (trans->init())
    {
        ROS_INFO("connected to main board");
    }
    else
    {
        ROS_ERROR("oops!!! can't connect to main board");
        return;
    }

    ros::Duration(1).sleep(); // wait for device
    ROS_INFO("end sleep");

    frame->init();
    // 底盘初始化
    init_chassis();
    ros::Duration(0.3).sleep();
    init_chassis();
    // 初始化里程计
    init_cmd_odom();
    init_sensor();
    init_joint();
}

BaseDriver::~BaseDriver()
{
    if (instance != NULL)
        delete instance;
}

// 设置底盘形态
void BaseDriver::init_chassis()
{
    ROS_INFO_STREAM("chassiss_type [" << bdg.chassis_type << "]");
    Data_holder::get()->chassiss_type.type = bdg.chassis_type;
    frame->interact(ID_SET_ROBOT_CHASSIS_TYPE);
}

// 初始化底盘
void BaseDriver::init_cmd_odom()
{
    //先给里程计清零
    Data_holder::get()->clear_odom.clear = 1;
    frame->interact(ID_CLEAR_ODOM);

    ROS_INFO_STREAM("subscribe cmd topic on [" << bdg.cmd_vel_topic << "]");
    cmd_vel_sub = nh.subscribe(bdg.cmd_vel_topic, 1000, &BaseDriver::cmd_vel_callback, this);

    ROS_INFO_STREAM("advertise odom topic on [" << bdg.odom_topic << "]");
    odom_pub = nh.advertise<nav_msgs::Odometry>(bdg.odom_topic, 50);

    cmd_single_servo_sub = nh.subscribe(bdg.cmd_single_servo_topic, 1000, &BaseDriver::cmd_single_servo_callback, this);
    cmd_multiple_servo_sub = nh.subscribe(bdg.cmd_multiple_servo_topic, 1000, &BaseDriver::cmd_multiple_servo_callback, this);

    last_left_encoder = 0.0;
    last_right_encoder = 0.0;
    last_yaw = 0.0;
    last_time = ros::Time::now();
    is_first_measurement = true;
    imu_sub = nh.subscribe(bdg.imu_topic, 1000, &BaseDriver::imu_data_callback, this);

    // init odom_trans
    odom_trans.header.frame_id = bdg.odom_frame;
    odom_trans.child_frame_id = bdg.base_frame;
    odom_trans.transform.translation.z = 0;

    // init odom
    odom.header.frame_id = bdg.odom_frame;
    odom.child_frame_id = bdg.base_frame;

    if (!bdg.publish_tf)
    {
        ROS_INFO_STREAM("init odom covariance");

        odom.pose.covariance = boost::assign::list_of(1e-3)(0)(0)(0)(0)(0)(0)(1e-3)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e3);

        odom.twist.covariance = boost::assign::list_of(1e-3)(0)(0)(0)(0)(0)(0)(1e-3)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e3);
    }

    need_update_speed = false;
}


// 初始化超声波传感器、TOF传感器信息
void BaseDriver::init_sensor()
{
    ul_sensor_pub1 = nh.advertise<sensor_msgs::Range>("/ul/sensor1", 50);
    ul_sensor_pub2 = nh.advertise<sensor_msgs::Range>("/ul/sensor2", 50);
    ul_sensor_pub3 = nh.advertise<sensor_msgs::Range>("/ul/sensor3", 50);
    ul_sensor_pub4 = nh.advertise<sensor_msgs::Range>("/ul/sensor4", 50);
    tof_pub1 = nh.advertise<sensor_msgs::Range>("/us/tof1", 50);
    tof_pub2 = nh.advertise<sensor_msgs::Range>("/us/tof2", 50);
    tof_pub3 = nh.advertise<sensor_msgs::Range>("/us/tof3", 50);
    tof_pub4 = nh.advertise<sensor_msgs::Range>("/us/tof4", 50);
    bump_sensor_pub = nh.advertise<std_msgs::Int16MultiArray>("/robot/bump_sensor", 50);

    ul_sensor1.radiation_type = 0;
    ul_sensor1.field_of_view = 0.3; // 角度范围
    ul_sensor1.max_range = 1.5;     // 最大范围
    ul_sensor1.min_range = 0.08;    // 最小范围

    ul_sensor2.radiation_type = 0;
    ul_sensor2.field_of_view = 0.3; // 角度范围
    ul_sensor2.max_range = 1.5;     // 最大范围
    ul_sensor2.min_range = 0.08;    // 最小范围

    ul_sensor3.radiation_type = 0;
    ul_sensor3.field_of_view = 0.3; // 角度范围
    ul_sensor3.max_range = 1.5;     // 最大范围
    ul_sensor3.min_range = 0.08;    // 最小范围

    ul_sensor4.radiation_type = 0;
    ul_sensor4.field_of_view = 0.3; // 角度范围
    ul_sensor4.max_range = 1.5;     // 最大范围
    ul_sensor4.min_range = 0.08;    // 最小范围

    tof1.radiation_type = 1;
    tof1.field_of_view = 0.3; // 角度范围
    tof1.max_range = 1.5;     // 最大范围
    tof1.min_range = 0.08;    // 最小范围

    tof2.radiation_type = 1;
    tof2.field_of_view = 0.3; // 角度范围
    tof2.max_range = 1.5;     // 最大范围
    tof2.min_range = 0.08;    // 最小范围

    tof3.radiation_type = 1;
    tof3.field_of_view = 0.3; // 角度范围
    tof3.max_range = 1.5;     // 最大范围
    tof3.min_range = 0.08;    // 最小范围

    tof4.radiation_type = 1;
    tof4.field_of_view = 0.3; // 角度范围
    tof4.max_range = 1.5;     // 最大范围
    tof4.min_range = 0.08;    // 最小范围
}

// 初始化机械臂关节信息
void BaseDriver::init_joint()
{
    joint_pub = nh.advertise<sensor_msgs::JointState>(bdg.joint_state_topic, 50);
    joint_info.header.frame_id = "";
    joint_info.name.resize(5);
    joint_info.name[0] = "joint_1";
    joint_info.name[1] = "joint_2";
    joint_info.name[2] = "joint_3";
    joint_info.name[3] = "joint_4";
    joint_info.name[4] = "joint_5";
    joint_info.position.resize(5);
    joint_info.velocity.resize(5);
    joint_info.effort.resize(5);
}

// 控制小车移动
void BaseDriver::cmd_vel_callback(const geometry_msgs::Twist &vel_cmd)
{
    // ROS_INFO_STREAM("cmd_vel:[" << vel_cmd.linear.x << " " << vel_cmd.linear.y << " " << vel_cmd.angular.z << "]");
    int id_number1 = vel_cmd.angular.x;
    int id_number2 = vel_cmd.angular.y;
    // ROS_INFO_STREAM("idnumber:[" <<id_number1<<" "<< id_number2<< "]");

    Data_holder::get()->velocity.v_liner_x = vel_cmd.linear.x * 1000;
    Data_holder::get()->velocity.v_liner_y = vel_cmd.linear.y * 1000;
    Data_holder::get()->velocity.v_angular_z = vel_cmd.angular.z * 100;
    need_update_speed = true;
}

// 单个舵机控制信息回调
void BaseDriver::cmd_single_servo_callback(const upros_message::SingleServo &servoData)
{
    uint8_t id = servoData.ID;
    int16_t angle = servoData.Target_position_Angle;
    uint16_t speed = servoData.Rotation_Speed;
    Data_holder::get()->single_servo.ID = id;
    Data_holder::get()->single_servo.Target_position_Angle = angle;
    Data_holder::get()->single_servo.Rotation_Speed = speed;
    frame->interact(ID_SINGLE_SERVO);
}

// 多个舵机控制信息回调
void BaseDriver::cmd_multiple_servo_callback(const upros_message::MultipleServo &servoData)
{
    int i = 0;
    for (std::vector<upros_message::SingleServo>::const_iterator it = servoData.servo_gather.begin(); it != servoData.servo_gather.end(); it++)
    {
        uint8_t id = it->ID;
        int16_t angle = it->Target_position_Angle;
        uint16_t speed = it->Rotation_Speed;
        Single_Servo single_Servo;
        single_Servo.ID = id;
        single_Servo.Target_position_Angle = angle;
        single_Servo.Rotation_Speed = speed;
        ROS_INFO("id=%d,angle=%d,speed=%d", id, angle, speed);
        Data_holder::get()->multiple_servo.servo_gather[i].ID = id;
        Data_holder::get()->multiple_servo.servo_gather[i].Target_position_Angle = angle;
        Data_holder::get()->multiple_servo.servo_gather[i].Rotation_Speed = speed;
        i++;
    }
    frame->interact(ID_MULTPLE_SERVO);
}

// imu数据回调
void BaseDriver::imu_data_callback(const sensor_msgs::Imu &imu_data)
{
    //获取当前imu姿态角
    current_yaw = tf::getYaw(imu_data.orientation);
    //当前姿态设定为初始姿态
    if (is_first_measurement)
    {
        first_error_yaw = current_yaw;
        last_yaw = current_yaw;
        is_first_measurement = false;
        return;
    }
}

void BaseDriver::work_loop()
{

    ros::Rate loop(1000);
    while (ros::ok())
    {
        boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
        
        //异步更新传感器
        if (!(clock % devide_clock_senser))
        {
            update_sensor();
        }
        
        //异步更新碰撞
        if (!(clock % devide_clock_senser_status))
        {
            update_sensor_status();
        }

        //异步更新里程计
        if (!(clock % devide_clock_odom))
        {
            update_motor_encoder();
        }
        
        //发布各个关节角度
        if (bdg.publish_joint_state)
        {
            if (!(clock % device_clock_joint_info))
            {
                update_joint_info();
            }
        }

        if (!(clock % devide_clock_speed))
        {
            update_speed();
        }

        clock++;
        if (clock >= 30000)
        {
            clock = 0;
        }

        frame->recv_proc();
        loop.sleep();
        ros::spinOnce();
    }
}

// 下发速度控制指令
void BaseDriver::update_speed()
{
    if (need_update_speed)
    {
        frame->interact(ID_SET_VELOCITY);
        need_update_speed = false;
    }
}

// 发布超声波传感器、tof传感器信息
void BaseDriver::update_sensor()
{
    frame->interact(ID_UL_SENSOR);
    ul_sensor1.header.stamp = ros::Time::now();
    ul_sensor2.header.stamp = ros::Time::now();
    ul_sensor3.header.stamp = ros::Time::now();
    ul_sensor4.header.stamp = ros::Time::now();

    tof1.header.stamp = ros::Time::now();
    tof2.header.stamp = ros::Time::now();
    tof3.header.stamp = ros::Time::now();
    tof4.header.stamp = ros::Time::now();

    ul_sensor1.header.frame_id = "ul_sensor1";
    ul_sensor2.header.frame_id = "ul_sensor2";
    ul_sensor3.header.frame_id = "ul_sensor3";
    ul_sensor4.header.frame_id = "ul_sensor4";

    tof1.header.frame_id = "tof1";
    tof2.header.frame_id = "tof2";
    tof3.header.frame_id = "tof3";
    tof4.header.frame_id = "tof4";

    ul_sensor1.range = Data_holder::get()->ul_sensor.ul1 / 1000.0;
    ul_sensor2.range = Data_holder::get()->ul_sensor.ul2 / 1000.0;
    ul_sensor3.range = Data_holder::get()->ul_sensor.ul3 / 1000.0;
    ul_sensor4.range = Data_holder::get()->ul_sensor.ul4 / 1000.0;

    tof1.range = Data_holder::get()->ul_sensor.tof1 / 1000.0;
    tof2.range = Data_holder::get()->ul_sensor.tof2 / 1000.0;
    tof3.range = Data_holder::get()->ul_sensor.tof3 / 1000.0;
    tof4.range = Data_holder::get()->ul_sensor.tof4 / 1000.0;

    ul_sensor_pub1.publish(ul_sensor1);
    ul_sensor_pub2.publish(ul_sensor2);
    ul_sensor_pub3.publish(ul_sensor3);
    ul_sensor_pub4.publish(ul_sensor4);

    tof_pub1.publish(tof1);
    tof_pub2.publish(tof2);
    tof_pub3.publish(tof3);
    tof_pub4.publish(tof4);
}

// 获取碰撞传感器状态
void BaseDriver::update_sensor_status()
{
    frame->interact(ID_SENSRO_STATUS);
    uint8_t collision = Data_holder::get()->sensor_status.collision;
    uint8_t c1 = (collision >> 0) & 1;
    uint8_t c2 = (collision >> 1) & 1;
    uint8_t c3 = (collision >> 2) & 1;
    uint8_t c4 = (collision >> 3) & 1;
    std::vector<int16_t> array({c1, c2, c3, c4});
    bump_sensor_array.data = array;
    bump_sensor_pub.publish(bump_sensor_array);
}

// 获取底盘电机编码器的值
void BaseDriver::update_motor_encoder()
{
    frame->interact(ID_GET_MOTOR_ENCODER);
    int32 motor1 = Data_holder::get()->encoder.motor1;
    int32 motor2 = Data_holder::get()->encoder.motor2;
    int32 motor3 = Data_holder::get()->encoder.motor3;
    int32 motor4 = Data_holder::get()->encoder.motor4;

    if (bdg.chassis_type == 1)
    {
        update_mec_odom(motor1, motor2, motor3, motor4);
    }
    if (bdg.chassis_type == 2)
    {
        update_onmi_odom(motor1, -motor2, -motor3);
    }
    if (bdg.chassis_type == 3)
    {
        update_diff_odom(motor1, motor2);
    }

    last_motor_1 = motor1;
    last_motor_2 = motor2;
    last_motor_3 = motor3;
    last_motor_4 = motor4;
}

// 计算两轮差速底盘里程计
void BaseDriver::update_diff_odom(int32 motor1_encoder, int32 motor2_encoder)
{
    // ROS_INFO("motor: encoder1=%d encoder2=%d", motor1_encoder, motor2_encoder);
    // 计算两个轮子走过的距离
    float left_encoder = float(motor1_encoder / bdg.motor_encoder / bdg.motor_ratio) * M_PI * bdg.diff_wheel_radius;
    float right_encoder = float(motor2_encoder / bdg.motor_encoder / bdg.motor_ratio) * M_PI * bdg.diff_wheel_radius;
    // 计算两个轮子的距离增量
    float delta_left_encoder = left_encoder - last_left_encoder;
    float delta_right_encoder = right_encoder - last_right_encoder;
    // 底盘距离增量
    float delta_distance = (delta_left_encoder + delta_right_encoder) / 2.0;
    // imu航向角
    float imu_yaw = current_yaw - first_error_yaw;
    // imu航向角变化角度的增量
    float delta_yaw = imu_yaw - last_yaw;
    // 时间
    ros::Time current_time = ros::Time::now();
    float delta_time = (current_time - last_time).toSec();
    // 每次里程计的xy轴的增量
    float delta_x = delta_distance * cos(imu_yaw);
    float delta_y = delta_distance * sin(imu_yaw);
    // 里程计速度
    float vx = delta_x / delta_time;
    float vy = 0.0;
    float vth = delta_yaw / delta_time;

    last_left_encoder = left_encoder;
    last_right_encoder = right_encoder;
    last_yaw = imu_yaw;
    last_time = current_time;

    // 里程计xy
    pos_x += delta_x;
    pos_y += delta_y;

    // 里程计消息
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(imu_yaw);

    if (bdg.publish_tf)
    {
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = pos_x;
        odom_trans.transform.translation.y = pos_y;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);
    }
    odom.header.stamp = current_time;
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;
    odom_pub.publish(odom);
    // ROS_INFO("odom: x=%.2f y=%.2f yaw=%.8f", pos_x, pos_y ,imu_yaw);
}

// 计算三轮全向底盘里程计
void BaseDriver::update_onmi_odom(int32 motor1_encoder, int32 motor2_encoder, int32 motor3_encoder)
{
    // 计算三个轮子走过的距离
    float encoder1 = motor1_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.onmi_wheel_radius;
    float encoder2 = motor2_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.onmi_wheel_radius;
    float encoder3 = motor3_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.onmi_wheel_radius;

    // 计算三个轮子的距离增量
    float delta_motor1_encoder = encoder1 - last_encoder1;
    float delta_motor2_encoder = encoder2 - last_encoder2;
    float delta_motor3_encoder = encoder3 - last_encoder3;

    // 底盘距离增量
    float delta_distance_x = (delta_motor1_encoder - delta_motor2_encoder) / sqrt(3.0f);
    float delta_distance_y = (2 * delta_motor3_encoder - delta_motor2_encoder - delta_motor1_encoder) / 3.0f;

    // imu航向角
    float imu_yaw = current_yaw - first_error_yaw;
    // imu航向角变化角度的增量
    float delta_yaw = imu_yaw - last_yaw;
    // 时间
    ros::Time current_time = ros::Time::now();
    float delta_time = (current_time - last_time).toSec();

    // 每次里程计的xy轴的增量
    float delta_x = delta_distance_x * cos(imu_yaw) - delta_distance_y * sin(imu_yaw);
    float delta_y = delta_distance_x * sin(imu_yaw) + delta_distance_y * cos(imu_yaw);
    // 里程计速度
    float vx = delta_x / delta_time;
    float vy = delta_y / delta_time;
    float vth = delta_yaw / delta_time;

    last_encoder1 = encoder1;
    last_encoder2 = encoder2;
    last_encoder3 = encoder3;
    last_yaw = imu_yaw;
    last_time = current_time;

    // 里程计xy
    pos_x += delta_x;
    pos_y += delta_y;

    // 里程计消息
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(imu_yaw);

    if (bdg.publish_tf)
    {
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = pos_x;
        odom_trans.transform.translation.y = pos_y;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);
    }
    odom.header.stamp = current_time;
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.x = vy;
    odom.twist.twist.angular.z = vth;
    odom_pub.publish(odom);
    // ROS_INFO("odom: x=%.2f y=%.2f yaw=%.2f", pos_x, pos_y ,imu_yaw);
}

// 计算四轮全向底盘里程计
void BaseDriver::update_mec_odom(int32 motor1_encoder, int32 motor2_encoder, int32 motor3_encoder, int32 motor4_encoder)
{
    // 计算四个轮子走过的距离
    float encoder1 = motor1_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.mec_wheel_radius;
    float encoder2 = motor2_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.mec_wheel_radius;
    float encoder3 = motor3_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.mec_wheel_radius;
    float encoder4 = motor4_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.mec_wheel_radius;

    // 计算四个轮子的距离增量
    float delta_motor1_encoder = encoder1 - last_encoder1;
    float delta_motor2_encoder = encoder2 - last_encoder2;
    float delta_motor3_encoder = encoder3 - last_encoder3;
    float delta_motor4_encoder = encoder4 - last_encoder4;

    // 底盘距离增量
    float delta_distance_x = (delta_motor1_encoder + delta_motor2_encoder + delta_motor3_encoder + delta_motor4_encoder) / 4;
    float delta_distance_y = (delta_motor2_encoder + delta_motor4_encoder - delta_motor1_encoder - delta_motor3_encoder) / 4;

    // imu航向角
    float imu_yaw = current_yaw - first_error_yaw;
    // imu航向角变化角度的增量
    float delta_yaw = imu_yaw - last_yaw;
    // 时间
    ros::Time current_time = ros::Time::now();
    float delta_time = (current_time - last_time).toSec();

    // 每次里程计的xy轴的增量
    float delta_x = delta_distance_x * cos(imu_yaw) - delta_distance_y * sin(imu_yaw);
    float delta_y = delta_distance_x * sin(imu_yaw) + delta_distance_y * cos(imu_yaw);
    // 里程计速度
    float vx = delta_x / delta_time;
    float vy = delta_y / delta_time;
    float vth = delta_yaw / delta_time;

    last_encoder1 = encoder1;
    last_encoder2 = encoder2;
    last_encoder3 = encoder3;
    last_encoder4 = encoder4;
    last_yaw = imu_yaw;
    last_time = current_time;

    // 里程计xy
    pos_x += delta_x;
    pos_y += delta_y;

    // 里程计消息
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(imu_yaw);

    if (bdg.publish_tf)
    {
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = pos_x;
        odom_trans.transform.translation.y = pos_y;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);
    }
    odom.header.stamp = current_time;
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.x = vy;
    odom.twist.twist.angular.z = vth;
    odom_pub.publish(odom);
}

// 获取机械臂关节信息
void BaseDriver::update_joint_info()
{
    Data_holder::get()->servo_ids.joint1 = bdg.joint1;
    Data_holder::get()->servo_ids.joint2 = bdg.joint2;
    Data_holder::get()->servo_ids.joint3 = bdg.joint3;
    Data_holder::get()->servo_ids.joint4 = bdg.joint4;
    Data_holder::get()->servo_ids.joint5 = bdg.joint5;
    Data_holder::get()->servo_ids.claw_joint = bdg.claw_joint;
    frame->interact(ID_GET_SERVO_INFO);
    ros::Time current_time = ros::Time::now();
    joint_info.header.stamp = current_time;

    float joint1_pos = Data_holder::get()->servo_pos.servo_pos[0] / 10.0f / 180.0f * M_PI;
    float joint2_pos = Data_holder::get()->servo_pos.servo_pos[1] / 10.0f / 180.0f * M_PI;
    float joint3_pos = Data_holder::get()->servo_pos.servo_pos[2] / 10.0f / 180.0f * M_PI;
    float joint4_pos = Data_holder::get()->servo_pos.servo_pos[3] / 10.0f / 180.0f * M_PI;
    float joint5_pos = Data_holder::get()->servo_pos.servo_pos[4] / 10.0f / 180.0f * M_PI;
    float claw_pos = Data_holder::get()->servo_pos.servo_pos[5] / 10.0f / 180.0f * M_PI;

    joint_info.position[0] = joint1_pos;
    joint_info.position[1] = joint2_pos;
    joint_info.position[2] = joint3_pos;
    joint_info.position[3] = joint4_pos;
    joint_info.position[4] = joint5_pos;

    float delta_time = (current_time - last_joint_time).toSec();

    joint_info.velocity[0] = (joint1_pos - last_joint1_pos) / delta_time;
    joint_info.velocity[1] = (joint2_pos - last_joint2_pos) / delta_time;
    joint_info.velocity[2] = (joint3_pos - last_joint3_pos) / delta_time;
    joint_info.velocity[3] = (joint4_pos - last_joint4_pos) / delta_time;
    joint_info.velocity[4] = (joint5_pos - last_joint5_pos) / delta_time;

    last_joint1_pos = joint1_pos;
    last_joint2_pos = joint2_pos;
    last_joint3_pos = joint3_pos;
    last_joint4_pos = joint4_pos;
    last_joint5_pos = joint5_pos;
    last_claw_pos = claw_pos;
    last_joint_time = current_time;
    joint_pub.publish(joint_info);
}
