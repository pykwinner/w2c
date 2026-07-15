#ifndef ZOO_DATA_FRAME_H_
#define ZOO_DATA_FRAME_H_

enum MESSAGE_ID
{
    ID_SET_ROBOT_CHASSIS_TYPE = 0x01,
    ID_INIT_ODOM = 0x03,
    ID_SET_VELOCITY = 0x04,
    ID_GET_ODOM = 0x05,
    ID_CLEAR_ODOM = 0x06,
    ID_GET_IMU = 0X07,
    ID_SINGLE_SERVO = 0x08,
    ID_MULTPLE_SERVO = 0x09,
    ID_UL_SENSOR = 0X0C,
    ID_SENSRO_STATUS = 0X0D,
    ID_GET_MOTOR_ENCODER = 0X0F,
    ID_GET_SERVO_INFO = 0X10,
    ID_MESSGAE_MAX
};

class Notify
{
public:
    virtual void update(const MESSAGE_ID id, void *data) = 0;
};

class Dataframe
{
public:
    virtual bool init() = 0;
    virtual void register_notify(const MESSAGE_ID id, Notify *_nf) = 0;
    virtual bool data_recv(unsigned char c) = 0;
    virtual bool data_parse() = 0;
    virtual bool interact(const MESSAGE_ID id) = 0;
    virtual bool recv_proc() = 0;
};

#endif
