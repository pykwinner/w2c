#include "handsfree_ros_imu/imu_calibrate.h"
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>

ROSIMU::ROSIMU(ros::NodeHandle nh, ros::NodeHandle pnh):
  //  Members default values
  nh_(nh),
  pnh_(pnh),
  perform_calibration_(true),
  is_calibrated_(false)
{
  raw_sub_ = nh_.subscribe("/ros/imu", 5, &ROSIMU::rawCallback, this);
  imu_cal_srv_ = nh_.advertiseService("imu/calibrate_imu", &ROSIMU::calibrateCallback, this);

  //if not calibrate , calibrate it
  if (!pnh_.getParam("imu/accelerometer_bias", acceleration_bias_) || 
      !pnh_.getParam("imu/gyroscope_bias", gyroscope_bias_) ||
      !pnh_.getParam("imu/yaw_bias", yaw_bias_) )
  {
    ROS_WARN("IMU calibration NOT found.");
    is_calibrated_ = false;
  }
  else
  {
    ROS_INFO("IMU calibration found.");
    pnh_.getParam("imu/accelerometer_bias", acceleration_bias_);
    pnh_.getParam("imu/gyroscope_bias", gyroscope_bias_);
    is_calibrated_ = true;
  }

  //sample 100 data for calibrate
  pnh_.param<int>("imu/calibration_samples", calibration_samples_, 100);

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data", 1000);

  pnh_.param<double>("imu/linear_acc_stdev", linear_acc_stdev_, 0.0);
  ROSIMU::fillRowMajor(linear_acc_covar_, linear_acc_stdev_);
  pnh_.param<double>("imu/angular_vel_stdev", angular_vel_stdev_, 0.0);
  ROSIMU::fillRowMajor(angular_vel_covar_, angular_vel_stdev_);
  pnh_.param<double>("imu/orientation_stdev", magnetic_field_stdev_, 0.0);
  ROSIMU::fillRowMajor(orientation_covar_, magnetic_field_stdev_);

  ROS_INFO("Starting Raw Imu Bridge.");
}


void ROSIMU::rawCallback(const sensor_msgs::Imu::ConstPtr& raw_msg)
{
  if (perform_calibration_ || !is_calibrated_)
  {
    ROS_WARN_ONCE("Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");

    static int taken_samples;
    
    //记录前100组数据，累加误差
    if (taken_samples < calibration_samples_)
    {
      //calc acc
      acceleration_bias_["x"] += raw_msg->linear_acceleration.x;
      acceleration_bias_["y"] += raw_msg->linear_acceleration.y;
      acceleration_bias_["z"] += raw_msg->linear_acceleration.z;
      
      //calc gyro
      gyroscope_bias_["x"] += raw_msg->angular_velocity.x;
      gyroscope_bias_["y"] += raw_msg->angular_velocity.y;
      gyroscope_bias_["z"] += raw_msg->angular_velocity.z;
      
      //calc yaw
      double roll,pitch,yaw;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(raw_msg->orientation,quat);
      tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);//transform
      yaw_bias_["yaw"] += yaw;

      taken_samples++;
    }
    //记录满足100个数了，求平均值作为零漂
    else
    {
      acceleration_bias_["x"] /= calibration_samples_;
      acceleration_bias_["y"] /= calibration_samples_;
      acceleration_bias_["z"] = acceleration_bias_["z"] / calibration_samples_;

      gyroscope_bias_["x"] /= calibration_samples_;
      gyroscope_bias_["y"] /= calibration_samples_;
      gyroscope_bias_["z"] /= calibration_samples_;

      yaw_bias_["yaw"] /= calibration_samples_;

      ROS_INFO("Calibrating accelerometer and gyroscope complete.");
      ROS_INFO("Yaw_bias: %f ", yaw_bias_["yaw"]);

      pnh_.setParam("imu/accelerometer_bias", acceleration_bias_);
      pnh_.setParam("imu/gyroscope_bias", gyroscope_bias_);
      pnh_.setParam("imu/yaw_bias", yaw_bias_);

      is_calibrated_ = true;
      perform_calibration_ = false;
      taken_samples = 0;
      last_time_stamped_to_sec = raw_msg->header.stamp.toSec();
    }
  }
  //再后续的数据上，减去零漂
  else
  {
      sensor_msgs::ImuPtr imu_msg = boost::make_shared<sensor_msgs::Imu>();
      imu_msg->header = raw_msg->header;
      imu_msg->angular_velocity.x = raw_msg->angular_velocity.x ;
      imu_msg->angular_velocity.y = raw_msg->angular_velocity.y;
      imu_msg->angular_velocity.z = raw_msg->angular_velocity.z;
      imu_msg->orientation_covariance = angular_vel_covar_;

      imu_msg->linear_acceleration.x = raw_msg->linear_acceleration.x - acceleration_bias_["x"];
      imu_msg->linear_acceleration.y = raw_msg->linear_acceleration.y - acceleration_bias_["y"];
      imu_msg->linear_acceleration.z = raw_msg->linear_acceleration.z;
      imu_msg->linear_acceleration_covariance = linear_acc_covar_;

      //calc yaw
      double roll,pitch,yaw;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(raw_msg->orientation,quat);
      tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
      geometry_msgs::Quaternion yaw_quat;
      yaw_quat =tf::createQuaternionMsgFromYaw(yaw - yaw_bias_["yaw"]);
      imu_msg->orientation.x = 0.0;
      imu_msg->orientation.y = 0.0;
      imu_msg->orientation.z = yaw_quat.z;
      imu_msg->orientation.w = yaw_quat.w;
      imu_msg->orientation_covariance = orientation_covar_;
      
      //使用UTC时间，更加准确
      struct timeval tvt;
      gettimeofday(&tvt, 0);
      imu_msg->header.stamp.sec = tvt.tv_sec;
      imu_msg->header.stamp.nsec = tvt.tv_usec*1000;
      //publish imu data
      imu_pub_.publish(imu_msg);
  }
}

//收到服务，重新校准
bool ROSIMU::calibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_WARN("Calibrating accelerometer and gyroscope, make sure robot is stationary and level.");
  perform_calibration_ = true;
  return true;
}

void ROSIMU::fillRowMajor(boost::array<double, 9> & covar, double stdev)
{
  std::fill(covar.begin(), covar.end(), 0.0);
  covar[0] = pow(stdev, 2);  // X(roll)
  covar[4] = pow(stdev, 2);  // Y(pitch)
  covar[8] = pow(stdev, 2);  // Z(yaw)
}
