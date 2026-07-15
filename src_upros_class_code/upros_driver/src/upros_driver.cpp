#include "upros_driver/upros_driver.h"

UprosDriver *UprosDriver::instance = NULL;

UprosDriver::UprosDriver() : pn("~"), bdg(pn)
{
    // 初始化机器人形态配置
    bdg.init();

    // 声明数字通信类
    frame = boost::make_shared<DataFrame>();

    frame->init(bdg.port, bdg.buadrate);

    ros::Duration(1).sleep();
    ROS_INFO("BaseDriver startup");

    init_topic();

    init_sensor();

    if (bdg.publish_joint_state)
    {
        init_joint();
    }

    // 启动发送和接收线程
    frame->start_thread();

    // 底盘初始化，设置底盘形态，清除里程计
    init_chassis();
    ros::Duration(1.0).sleep();
}

UprosDriver::~UprosDriver()
{
    if (instance != NULL)
        delete instance;
}

void UprosDriver::init_topic()
{
    cmd_vel_sub = nh.subscribe(bdg.cmd_vel_topic, 1000, &UprosDriver::cmd_vel_callback, this);
    imu_sub = nh.subscribe(bdg.imu_topic, 1000, &UprosDriver::imu_data_callback, this);
    cmd_single_servo_sub = nh.subscribe(bdg.cmd_single_servo_topic, 1000, &UprosDriver::cmd_single_servo_callback, this);
    cmd_multiple_servo_sub = nh.subscribe(bdg.cmd_multiple_servo_topic, 1000, &UprosDriver::cmd_multiple_servo_callback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>(bdg.odom_topic, 50);
    servo_pose_pub = nh.advertise<std_msgs::Int16MultiArray>("/robot/servo_position", 50);
}

// 初始化超声波传感器、TOF传感器信息
void UprosDriver::init_sensor()
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

    ul_sensor1.header.frame_id = "ul_sensor1";
    ul_sensor2.header.frame_id = "ul_sensor2";
    ul_sensor3.header.frame_id = "ul_sensor3";
    ul_sensor4.header.frame_id = "ul_sensor4";

    tof1.header.frame_id = "tof1";
    tof2.header.frame_id = "tof2";
    tof3.header.frame_id = "tof3";
    tof4.header.frame_id = "tof4";

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

// 设置底盘形态，同时清空里程计
void UprosDriver::init_chassis()
{
    ROS_INFO_STREAM("chassiss_type [" << bdg.chassis_type << "]");
    int chassiss_type = bdg.chassis_type; // 底盘形态， 1 四轮全向底盘  2 三轮全向底盘  3 2轮差速底盘
    unsigned char data;
    if (chassiss_type == 1)
    {
        data = 0x03; // 0000 0011
    }
    else if (chassiss_type == 2)
    {
        data = 0x05; // 0000 0101
    }
    else if (chassiss_type == 3)
    {
        data = 0x07; // 0000 0111
    }
    unsigned char params[] = {data};
    frame->send_message(0x01, CMD_WRITE_RAM, Init_Chassis, params, sizeof(params));

    last_left_encoder = 0.0;
    last_right_encoder = 0.0;
    last_yaw = 0.0;
    last_time = ros::Time::now();
    is_first_measurement = true;

    // 初始化底盘里程计坐标变换
    odom_trans.header.frame_id = bdg.odom_frame;
    odom_trans.child_frame_id = bdg.base_frame;
    odom_trans.transform.translation.z = 0;

    // 初始化底盘里程计
    odom.header.frame_id = bdg.odom_frame;
    odom.pose.pose.position.z = 0.0;
    odom.child_frame_id = bdg.base_frame;
    odom.twist.twist.linear.y = 0;
    odom.pose.covariance = boost::assign::list_of(1e-3)(0)(0)(0)(0)(0)(0)(1e-3)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e3);
    odom.twist.covariance = boost::assign::list_of(1e-3)(0)(0)(0)(0)(0)(0)(1e-3)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e6)(0)(0)(0)(0)(0)(0)(1e3);
}

void UprosDriver::cmd_vel_callback(const geometry_msgs::Twist &vel_cmd)
{
    update_speed(vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.angular.z);
}

// imu数据回调
void UprosDriver::imu_data_callback(const sensor_msgs::Imu &imu_data)
{
    current_yaw = tf::getYaw(imu_data.orientation);
    if (is_first_measurement)
    {
        first_error_yaw = current_yaw;
        last_yaw = current_yaw;
        is_first_measurement = false;
        return;
    }
}

// 单个舵机控制信息回调
void UprosDriver::cmd_single_servo_callback(const upros_message::SingleServo &servoData)
{
    uint8_t id = servoData.ID;
    int16_t angle = servoData.Target_position_Angle;
    uint16_t speed = servoData.Rotation_Speed;
    // 协议打包
    unsigned char params[5];
    packServoData(id, angle, speed, params);
    frame->send_message(0x01, CMD_WRITE_SERVO_POS, Servo_Addr, params, sizeof(params));
}

// 多个舵机控制信息回调
void UprosDriver::cmd_multiple_servo_callback(const upros_message::MultipleServo &servoData)
{
    int i = 0;
    int servo_size = servoData.servo_gather.size() * 5;
    unsigned char params[servo_size];
    for (std::vector<upros_message::SingleServo>::const_iterator it = servoData.servo_gather.begin(); it != servoData.servo_gather.end(); it++)
    {
        uint8_t id = it->ID;
        int16_t angle = it->Target_position_Angle;
        uint16_t speed = it->Rotation_Speed;
        ROS_INFO("id=%d,angle=%d,speed=%d", id, angle, speed);
        // 协议打包
        unsigned char single_params[5];
        packServoData(id, angle, speed, single_params);
        for (int j = 0; j < 5; j++)
        {
            params[i * 5 + j] = single_params[j];
        }
        i++;
    }
    frame->send_message(0x01, CMD_WRITE_SERVO_POS, Servo_Addr, params, sizeof(params));
}

// 速度控制
void UprosDriver::update_speed(float x, float y, float yaw)
{
    std::vector<int16_t> vel;
    vel.push_back(int(x * 1000));
    vel.push_back(int(y * 1000));
    vel.push_back(int(yaw * 1000));
    unsigned char params[6];
    convertToUnsignedCharArray(vel, params);
    // 写入数组
    frame->send_message(0x01, CMD_WRITE_RAM, Set_Vel_Addr, params, sizeof(params));
}

// 将 vector<int16_t> 转换为 unsigned char 数组
void UprosDriver::convertToUnsignedCharArray(const std::vector<int16_t> &pos, unsigned char params[])
{
    size_t index = 0;
    // 遍历 vector 中的每个 int16_t 元素
    for (int32_t value : pos)
    {
        // 将 int16_t 元素拆分成 2 个字节
        for (int i = 0; i < 2; ++i)
        {
            // 提取当前字节
            params[index++] = static_cast<unsigned char>(value & 0xFF);
            // 右移 8 位，准备提取下一个字节
            value >>= 8;
        }
    }
}

// 将舵机ID，角度，速度打包成协议格式数组
void UprosDriver::packServoData(uint8_t id, int16_t angle, uint16_t speed, unsigned char params[])
{
    // ID (1 byte)
    params[0] = id;

    // Angle (2 bytes, int16_t -> uint16_t to handle sign correctly)
    uint16_t angle_unsigned = static_cast<uint16_t>(angle);
    params[2] = static_cast<unsigned char>((angle_unsigned >> 8) & 0xFF); // High byte
    params[1] = static_cast<unsigned char>(angle_unsigned & 0xFF);        // Low byte

    // Speed (2 bytes)
    params[4] = static_cast<unsigned char>((speed >> 8) & 0xFF); // High byte
    params[3] = static_cast<unsigned char>(speed & 0xFF);        // Low byte
}

void UprosDriver::work_loop()
{
    ros::Rate loop(30);
    while (ros::ok())
    {
        // 更新嵌入式上传的底盘信息，频率30HZ
        get_upros_datas();
        loop.sleep();
        if (bdg.publish_joint_state)
        {
            update_joint_info();
        }
        ros::spinOnce();
    }
}

void UprosDriver::get_upros_datas()
{
    // 所有的底盘信息，一共57字节
    unsigned char params[] = {0x39};

    // 发送查询指令
    frame->send_message(0x01, CMD_READ_RAM, Read_Addr, params, sizeof(params));

    // 接收嵌入式上传的信息，读取四路超声TOF的数据，并更新发布
    ul_sensor1.header.stamp = ros::Time::now();
    ul_sensor2.header.stamp = ros::Time::now();
    ul_sensor3.header.stamp = ros::Time::now();
    ul_sensor4.header.stamp = ros::Time::now();

    tof1.header.stamp = ros::Time::now();
    tof2.header.stamp = ros::Time::now();
    tof3.header.stamp = ros::Time::now();
    tof4.header.stamp = ros::Time::now();

    ul_sensor1.range = Data_holder::get()->sensor_status.sonar_1_range / 1000.0;
    ul_sensor2.range = Data_holder::get()->sensor_status.sonar_2_range / 1000.0;
    ul_sensor3.range = Data_holder::get()->sensor_status.sonar_3_range / 1000.0;
    ul_sensor4.range = Data_holder::get()->sensor_status.sonar_4_range / 1000.0;

    tof1.range = Data_holder::get()->sensor_status.tof_1_range / 1000.0;
    tof2.range = Data_holder::get()->sensor_status.tof_2_range / 1000.0;
    tof3.range = Data_holder::get()->sensor_status.tof_3_range / 1000.0;
    tof4.range = Data_holder::get()->sensor_status.tof_4_range / 1000.0;

    ul_sensor_pub1.publish(ul_sensor1);
    ul_sensor_pub2.publish(ul_sensor2);
    ul_sensor_pub3.publish(ul_sensor3);
    ul_sensor_pub4.publish(ul_sensor4);

    tof_pub1.publish(tof1);
    tof_pub2.publish(tof2);
    tof_pub3.publish(tof3);
    tof_pub4.publish(tof4);

    // 接收嵌入式上传的信息，读取四路碰撞的数据，并更新发布
    uint8_t collision = Data_holder::get()->sensor_status.collision;
    uint8_t c1 = (collision >> 0) & 1;
    uint8_t c2 = (collision >> 1) & 1;
    uint8_t c3 = (collision >> 2) & 1;
    uint8_t c4 = (collision >> 3) & 1;
    std::vector<int16_t> array({c1, c2, c3, c4});
    bump_sensor_array.data = array;
    bump_sensor_pub.publish(bump_sensor_array);

    // 接收嵌入式上传的信息，读取码盘的数据，并更新发布
    int32 motor1 = - Data_holder::get()->sensor_status.motor1;
    int32 motor2 = Data_holder::get()->sensor_status.motor2;
    int32 motor3 = Data_holder::get()->sensor_status.motor3;
    int32 motor4 = - Data_holder::get()->sensor_status.motor4;

    // ROS_INFO("Motor1: %d, Motor2: %d, Motor3: %d, Motor4: %d......", motor1, motor2, motor3, motor4);

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
}

// 计算两轮差速底盘里程计
void UprosDriver::update_diff_odom(int32 motor1_encoder, int32 motor2_encoder)
{
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
    // 记录上一帧的计算码盘
    last_left_encoder = left_encoder;
    last_right_encoder = right_encoder;
    last_yaw = imu_yaw;
    last_time = current_time;
    // 里程计xy
    pos_x += delta_x;
    pos_y += delta_y;
    // 里程计yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(imu_yaw);
    // 如果发布TF变换（一般不发布）
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
}

// 计算三轮全向底盘里程计
void UprosDriver::update_onmi_odom(int32 motor1_encoder, int32 motor2_encoder, int32 motor3_encoder)
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
}

// 计算四轮全向底盘里程计
void UprosDriver::update_mec_odom(int32 motor1_encoder, int32 motor2_encoder, int32 motor3_encoder, int32 motor4_encoder)
{
    // 计算四个轮子走过的距离
    float encoder1 = motor1_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.mec_wheel_radius;
    float encoder2 = motor2_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.mec_wheel_radius;
    float encoder3 = motor3_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.mec_wheel_radius;
    float encoder4 = motor4_encoder / bdg.motor_encoder / bdg.motor_ratio * M_PI * bdg.mec_wheel_radius;

    // ROS_INFO("Motor1: %d, Motor2: %d, Motor3: %d, Motor4: %d......", motor1_encoder, motor2_encoder, motor3_encoder, motor4_encoder);
    // ROS_INFO("encoder1: %f, encoder2: %f......", encoder1, encoder2);

    // 计算四个轮子的距离增量
    float delta_motor1_encoder = encoder1 - last_encoder1;
    float delta_motor2_encoder = encoder2 - last_encoder2;
    float delta_motor3_encoder = encoder3 - last_encoder3;
    float delta_motor4_encoder = encoder4 - last_encoder4;

    // 底盘距离增量
    float delta_distance_x = (delta_motor1_encoder + delta_motor2_encoder + delta_motor3_encoder + delta_motor4_encoder) / 4;
    float delta_distance_y = (delta_motor2_encoder + delta_motor4_encoder - delta_motor1_encoder - delta_motor3_encoder) / 4;
    // ROS_INFO("delta_distance_x: %f, delta_distance_y: %f......", delta_distance_x, delta_distance_y);

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

// 初始化机械臂关节信息
void UprosDriver::init_joint()
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

// 获取机械臂关节信息
void UprosDriver::update_joint_info()
{
    // 查询舵机信息，查询5个舵机的id，1，2，3，4，5
    unsigned char params[] = {1, 2, 3, 4, 5};

    // 发送查询指令
    frame->send_message(0x01, CMD_READ_SERVO_POS, Servo_Addr, params, sizeof(params));


    std::vector<int16_t> array;
    for (int i=0; i<5; i++)
    {
        array.push_back(Data_holder::get()->servo_pos.servo_pos[i]);
    }
    servo_pos_array.data = array;
    servo_pose_pub.publish(servo_pos_array);

    // 发送 joint 指令
    ros::Time current_time = ros::Time::now();
    joint_info.header.stamp = current_time;
    // 查询机械臂舵机
    float joint1_pos = Data_holder::get()->servo_pos.servo_pos[0] / 10.0f / 180.0f * M_PI;
    float joint2_pos = Data_holder::get()->servo_pos.servo_pos[1] / 10.0f / 180.0f * M_PI;
    float joint3_pos = Data_holder::get()->servo_pos.servo_pos[2] / 10.0f / 180.0f * M_PI;
    float joint4_pos = Data_holder::get()->servo_pos.servo_pos[3] / 10.0f / 180.0f * M_PI;
    float joint5_pos = Data_holder::get()->servo_pos.servo_pos[4] / 10.0f / 180.0f * M_PI;

    // 数组赋值
    joint_info.position[0] = joint1_pos;
    joint_info.position[1] = joint2_pos;
    joint_info.position[2] = joint3_pos;
    joint_info.position[3] = joint4_pos;
    joint_info.position[4] = joint5_pos;

    // 计算时间差
    float delta_time = (current_time - last_joint_time).toSec();

    // 计算速度
    joint_info.velocity[0] = (joint1_pos - last_joint1_pos) / delta_time;
    joint_info.velocity[1] = (joint2_pos - last_joint2_pos) / delta_time;
    joint_info.velocity[2] = (joint3_pos - last_joint3_pos) / delta_time;
    joint_info.velocity[3] = (joint4_pos - last_joint4_pos) / delta_time;
    joint_info.velocity[4] = (joint5_pos - last_joint5_pos) / delta_time;

    // 记录上一帧
    last_joint1_pos = joint1_pos;
    last_joint2_pos = joint2_pos;
    last_joint3_pos = joint3_pos;
    last_joint4_pos = joint4_pos;
    last_joint5_pos = joint5_pos;
    last_joint_time = current_time;

    // 发布机械臂关节
    joint_pub.publish(joint_info);
}
