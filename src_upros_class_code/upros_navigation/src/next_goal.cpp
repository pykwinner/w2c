#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>

#include <actionlib/client/simple_action_client.h>
#include "move_base_msgs/MoveBaseAction.h"

using namespace std;
using namespace tf;

#define NEXT_PIONT_PERIOD 1.0 // 两目标点之间的时间间隔

float x_current;
float y_current;

float normeNextGoal = 0.25;

int PointCnt;
bool goal_reached;
bool goal_sent;

class quaternion_ros
{
public:
  float w;
  float x;
  float y;
  float z;

  quaternion_ros();

  void toQuaternion(float pitch, float roll, float yaw);
};

void quaternion_ros::toQuaternion(float pitch, float roll, float yaw)
{

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  w = cy * cr * cp + sy * sr * sp;
  x = cy * sr * cp - sy * cr * sp;
  y = cy * cr * sp + sy * sr * cp;
  z = sy * cr * cp - cy * sr * sp;
}

quaternion_ros::quaternion_ros()
{
  w = 1;
  x = 0;
  y = 0;
  z = 0;
}

class Path_planned
{
public:
  struct Goal
  {
    float x;
    float y;
    bool visited;
  };

  vector<Goal> Path;

  Path_planned();

  void addGoal(float X, float Y, bool visit);
};

Path_planned::Path_planned()
{
}

void Path_planned::addGoal(float X, float Y, bool visit)
{
  Path_planned::Goal newGoal;
  newGoal.x = X;
  newGoal.y = Y;
  newGoal.visited = visit;
  Path.push_back(newGoal);
}

Path_planned planned_path;
nav_msgs::Path passed_path;
ros::Publisher pub_passed_path;

void pose_callback(const nav_msgs::Odometry &poses)
{ 
  // 里程计回调函数,用来计算当前机器人位置与前面目标点的距离,判断是否要发新的幕摆点
  x_current = poses.pose.pose.position.x;
  y_current = poses.pose.pose.position.y;
  passed_path.header = poses.header;
  geometry_msgs::PoseStamped p;
  p.header = poses.header;
  p.pose = poses.pose.pose;
  passed_path.poses.emplace_back(p);
  pub_passed_path.publish(passed_path);
}

int taille_last_path = 0;
bool new_path = false;

// 接受规划的路径
void path_callback(const nav_msgs::Path &path)
{
  // 注意为了rviz显示方便 路径一直在发,但是这里只用接受一次就好,当规划的路径发生变化时候再重新装载
  if ((planned_path.Path.size() == 0) || (path.poses.size() != taille_last_path))
  {
    planned_path.Path.clear();
    new_path = true;
    for (int i = 0; i < path.poses.size(); i++)
    {
      planned_path.addGoal(path.poses[i].pose.position.x, path.poses[i].pose.position.y, false);

      cout << path.poses[i].pose.position.x << " " << path.poses[i].pose.position.y << endl;
    }
    cout << "Recv path size:" << path.poses.size() << endl;
    taille_last_path = path.poses.size();
  }
}

void doneCb()
{
  goal_reached = true;
  goal_sent = false;
  PointCnt++;
}

void activeCb()
{
}

void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
}

int main(int argc, char *argv[])
{
  //srand(time(0));

  ros::init(argc, argv, "next_goal");

  ros::NodeHandle next_goal;

  ros::Subscriber sub1 = next_goal.subscribe("/odom", 10, pose_callback);

  int PATH_TYPE;
  next_goal.param<int>("path_type", PATH_TYPE, 0);

  ros::Subscriber sub2;
  if (PATH_TYPE == 0) // 扫描线法为
  {
    sub2 = next_goal.subscribe("/area_path_node/area_path", 10, path_callback);
  }

  else if (PATH_TYPE == 1) // 栅格法
  {
    sub2 = next_goal.subscribe("/path_planning_node/cleaning_plan_nodehandle/cleaning_path", 10, path_callback);
  }

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base("move_base", true);

  move_base.waitForServer();

  move_base_msgs::MoveBaseGoal goal;

  pub_passed_path = next_goal.advertise<nav_msgs::Path>("/clean_robot/passed_path", 1000);

  ros::Rate loop_rate(10);

  geometry_msgs::PoseStamped goal_msgs;
  PointCnt = 0;
  double angle;
  goal_reached = false;
  goal_sent = false;

  // 获取发送下一个点的阈值
  if (!next_goal.getParam("/next_goal/tolerance_goal", normeNextGoal))
  {
    ROS_ERROR("Please set your tolerance_goal");
    return 0;
  }
  ROS_INFO("tolerance_goal=%f", normeNextGoal);

  double last_time = ros::Time::now().toSec();
  double x_last = x_current, y_last = y_current;
  while (ros::ok())
  {
    ros::spinOnce();
    if (new_path)
    {
      PointCnt = 0;
      new_path = false;
    }
    // 当前处理的点
    cout << " PointCnt : " << PointCnt << endl;
    if (!planned_path.Path.empty())
    {
      if (goal_reached == true || goal_sent == false)
      {

        goal_msgs.header.frame_id = "map";
        goal_msgs.header.stamp = ros::Time::now();
        goal_msgs.pose.position.x = planned_path.Path[PointCnt].x;
        goal_msgs.pose.position.y = planned_path.Path[PointCnt].y;
        goal_msgs.pose.position.z = 0;

        if (PointCnt < planned_path.Path.size())
        { 
          angle = atan2(planned_path.Path[PointCnt + 1].y - planned_path.Path[PointCnt].y, planned_path.Path[PointCnt + 1].x - planned_path.Path[PointCnt].x);
        }
        else
        {
          angle = atan2(planned_path.Path[0].y - planned_path.Path[PointCnt].y, planned_path.Path[0].x - planned_path.Path[PointCnt].x);
        }
        cout << angle << endl;
        quaternion_ros q;
        q.toQuaternion(0, 0, float(angle));
        goal_msgs.pose.orientation.w = q.w;
        goal_msgs.pose.orientation.x = q.x;
        goal_msgs.pose.orientation.y = q.y;
        if (planned_path.Path[PointCnt].x < planned_path.Path[PointCnt + 1].x)
        {
          goal_msgs.pose.orientation.z = 0;
        }
        if (planned_path.Path[PointCnt].x > planned_path.Path[PointCnt + 1].x)
        {
          goal_msgs.pose.orientation.z = 2;
        }

        cout << " NEW GOAL " << endl;
        cout << " x = " << planned_path.Path[PointCnt].x << " y = " << planned_path.Path[PointCnt].y << endl;

        goal_reached = false;
        goal_sent = true;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = planned_path.Path[PointCnt].x;
        goal.target_pose.pose.position.y = planned_path.Path[PointCnt].y;
        goal.target_pose.pose.orientation.x = goal_msgs.pose.orientation.x;
        goal.target_pose.pose.orientation.y = goal_msgs.pose.orientation.y;
        goal.target_pose.pose.orientation.z = goal_msgs.pose.orientation.z;
        goal.target_pose.pose.orientation.w = goal_msgs.pose.orientation.w;

        move_base.sendGoal(goal, boost::bind(&doneCb), boost::bind(&activeCb), boost::bind(&feedbackCb, _1));
      }
    }
    loop_rate.sleep();
  }

  return 0;
}
