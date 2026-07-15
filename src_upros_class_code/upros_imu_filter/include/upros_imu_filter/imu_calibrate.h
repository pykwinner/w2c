#ifndef _IMU_CALIBRATE_H_
#define _IMU_CALIBRATE_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>


class ROSIMU
{
  private:
    ros::NodeHandle nh_, pnh_;

    bool perform_calibration_, is_calibrated_;//flag to calc calibration and is_calibrated
    int calibration_samples_;

    //零漂参数
    std::map<std::string,double> acceleration_bias_, gyroscope_bias_, yaw_bias_;

    //置信矩阵
    double linear_acc_stdev_, angular_vel_stdev_, magnetic_field_stdev_;
    boost::array<double, 9> linear_acc_covar_;
    boost::array<double, 9> angular_vel_covar_;
    boost::array<double, 9> orientation_covar_;

    // Used for mag scaling
    double mag_x_min_, mag_x_max_;  //  [T]
    double mag_y_min_, mag_y_max_;
    double mag_z_min_, mag_z_max_;
    static constexpr double MILIGAUSS_TO_TESLA_SCALE = 0.0000001;  // From Milligauss [mG] to Tesla [T]

    //订阅和发布
    ros::Publisher imu_pub_;
    ros::Subscriber raw_sub_;

    // ROS services
    ros::ServiceServer imu_cal_srv_;

    //订阅和服务
    void rawCallback(const sensor_msgs::Imu::ConstPtr& raw_msg);
    bool calibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    void fillRowMajor(boost::array<double, 9> & covar, double stdev);

    double last_time_stamped_to_sec = 0;

  public:
    ROSIMU(ros::NodeHandle nh, ros::NodeHandle pnh);
    virtual ~ROSIMU(void){};
};

#endif  
