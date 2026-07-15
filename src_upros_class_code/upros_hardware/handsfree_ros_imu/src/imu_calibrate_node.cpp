#include "handsfree_ros_imu/imu_calibrate.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zyzx_imu");
  ros::NodeHandle nh, pnh("~");
  ROSIMU ros_imu(nh, pnh);

  ros::spin();

  return 0;
}