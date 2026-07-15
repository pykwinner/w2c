#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

namespace
{

const std::size_t EXPECTED_TARGET_COUNT = 9;

struct ObservationPoint
{
    double x;
    double y;
    double yaw;
};

bool isFinite(double value)
{
    return std::isfinite(value);
}

bool loadObservationPoints(
    ros::NodeHandle &private_nh,
    std::vector<ObservationPoint> &points)
{
    std::vector<double> target_x;
    std::vector<double> target_y;
    std::vector<double> target_yaw;

    if (!private_nh.getParam("target_x", target_x) ||
        !private_nh.getParam("target_y", target_y) ||
        !private_nh.getParam("target_yaw", target_yaw))
    {
        ROS_ERROR("target_x, target_y or target_yaw parameter is missing");
        return false;
    }

    if (target_x.size() != EXPECTED_TARGET_COUNT ||
        target_y.size() != EXPECTED_TARGET_COUNT ||
        target_yaw.size() != EXPECTED_TARGET_COUNT)
    {
        ROS_ERROR(
            "Target array size is invalid: x=%lu y=%lu yaw=%lu, expected=9",
            static_cast<unsigned long>(target_x.size()),
            static_cast<unsigned long>(target_y.size()),
            static_cast<unsigned long>(target_yaw.size()));
        return false;
    }

    points.clear();
    points.reserve(EXPECTED_TARGET_COUNT);
    bool all_zero = true;

    for (std::size_t i = 0; i < EXPECTED_TARGET_COUNT; ++i)
    {
        if (!isFinite(target_x[i]) ||
            !isFinite(target_y[i]) ||
            !isFinite(target_yaw[i]))
        {
            ROS_ERROR("Target %lu contains an invalid value",
                      static_cast<unsigned long>(i + 1));
            return false;
        }

        ObservationPoint point;
        point.x = target_x[i];
        point.y = target_y[i];
        point.yaw = target_yaw[i];
        points.push_back(point);

        if (std::fabs(point.x) > 1e-6 ||
            std::fabs(point.y) > 1e-6 ||
            std::fabs(point.yaw) > 1e-6)
        {
            all_zero = false;
        }
    }

    if (all_zero)
    {
        ROS_ERROR(
            "All target poses are zero. targets.yaml still contains placeholders");
        return false;
    }

    return true;
}

move_base_msgs::MoveBaseGoal createGoal(
    const std::string &frame_id,
    const ObservationPoint &point)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = frame_id;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = point.x;
    goal.target_pose.pose.position.y = point.y;
    goal.target_pose.pose.position.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, point.yaw);
    quaternion.normalize();
    goal.target_pose.pose.orientation.x = quaternion.x();
    goal.target_pose.pose.orientation.y = quaternion.y();
    goal.target_pose.pose.orientation.z = quaternion.z();
    goal.target_pose.pose.orientation.w = quaternion.w();
    return goal;
}

bool navigateToPoint(
    MoveBaseClient &client,
    const ObservationPoint &point,
    const std::string &frame_id,
    std::size_t point_number,
    double timeout,
    int retries)
{
    const int total_attempts = retries + 1;

    for (int attempt = 1;
         attempt <= total_attempts && ros::ok();
         ++attempt)
    {
        ROS_INFO(
            "Navigate to point %lu: x=%.3f y=%.3f yaw=%.3f, attempt=%d/%d",
            static_cast<unsigned long>(point_number),
            point.x, point.y, point.yaw, attempt, total_attempts);

        client.sendGoal(createGoal(frame_id, point));
        const bool finished = client.waitForResult(ros::Duration(timeout));

        if (!finished)
        {
            ROS_WARN("Navigation to point %lu timed out",
                     static_cast<unsigned long>(point_number));
            client.cancelGoal();
            client.waitForResult(ros::Duration(0.5));
        }
        else if (client.getState() ==
                 actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            client.cancelAllGoals();
            ros::Duration(0.20).sleep();
            return true;
        }
        else
        {
            ROS_WARN("Navigation to point %lu failed: %s",
                     static_cast<unsigned long>(point_number),
                     client.getState().toString().c_str());
        }

        if (attempt < total_attempts)
        {
            ros::Duration(0.50).sleep();
        }
    }

    return false;
}

bool ensureLaserClosed(
    ros::ServiceClient &close_client,
    const std::string &reason)
{
    if (!close_client.exists())
    {
        ROS_ERROR("/close is unavailable before %s", reason.c_str());
        return false;
    }

    std_srvs::Empty srv;
    if (!close_client.call(srv))
    {
        ROS_ERROR("Failed to close laser before %s", reason.c_str());
        return false;
    }
    return true;
}

bool runAprilTagShooting(
    ros::NodeHandle &nh,
    std::size_t point_number,
    const std::string &result_param)
{
    // 防止读取到上一靶留下的成功结果。
    nh.setParam(result_param, false);

    ROS_INFO("Start AprilTag shooting for target %lu",
             static_cast<unsigned long>(point_number));

    const int process_result =
        std::system("roslaunch shoot_robot shoot_tag_1.launch");

    bool shoot_succeeded = false;
    if (!nh.getParam(result_param, shoot_succeeded))
    {
        ROS_ERROR("Shoot result parameter was not produced for target %lu",
                  static_cast<unsigned long>(point_number));
        return false;
    }

    if (process_result != 0 || !shoot_succeeded)
    {
        ROS_ERROR(
            "Target %lu shooting failed: process_code=%d result=%s",
            static_cast<unsigned long>(point_number),
            process_result,
            shoot_succeeded ? "true" : "false");
        return false;
    }

    ROS_INFO("Target %lu shooting succeeded",
             static_cast<unsigned long>(point_number));
    return true;
}

} // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shoot_robot_controller");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string frame_id;
    std::string move_base_action;
    std::string close_service_name;
    std::string shoot_result_param;
    double move_base_server_timeout;
    double navigation_timeout;
    double close_service_wait_timeout;
    int navigation_retries;
    bool continue_after_navigation_failure;
    bool continue_after_shoot_failure;

    private_nh.param<std::string>("frame_id", frame_id, "map");
    private_nh.param<std::string>(
        "move_base_action", move_base_action, "/move_base");
    private_nh.param<std::string>(
        "close_service", close_service_name, "/close");
    private_nh.param<std::string>(
        "shoot_result_param",
        shoot_result_param,
        "/shoot_robot/last_shoot_success");
    private_nh.param(
        "move_base_server_timeout", move_base_server_timeout, 15.0);
    private_nh.param("navigation_timeout", navigation_timeout, 30.0);
    private_nh.param(
        "close_service_wait_timeout", close_service_wait_timeout, 5.0);
    private_nh.param("navigation_retries", navigation_retries, 1);
    private_nh.param(
        "continue_after_navigation_failure",
        continue_after_navigation_failure,
        true);
    private_nh.param(
        "continue_after_shoot_failure",
        continue_after_shoot_failure,
        true);

    move_base_server_timeout = std::max(1.0, move_base_server_timeout);
    navigation_timeout = std::max(1.0, navigation_timeout);
    close_service_wait_timeout = std::max(1.0, close_service_wait_timeout);
    navigation_retries = std::max(0, navigation_retries);

    std::vector<ObservationPoint> points;
    if (!loadObservationPoints(private_nh, points))
    {
        ROS_FATAL("Failed to load the 9 observation points");
        return 1;
    }

    ObservationPoint home;
    private_nh.param("home_x", home.x, 0.0);
    private_nh.param("home_y", home.y, 0.0);
    private_nh.param("home_yaw", home.yaw, 0.0);
    if (!isFinite(home.x) || !isFinite(home.y) || !isFinite(home.yaw))
    {
        ROS_FATAL("Home pose contains an invalid value");
        return 1;
    }

    MoveBaseClient move_base_client(move_base_action, true);
    if (!move_base_client.waitForServer(
            ros::Duration(move_base_server_timeout)))
    {
        ROS_FATAL("move_base action server is unavailable");
        return 1;
    }

    ros::ServiceClient close_client =
        nh.serviceClient<std_srvs::Empty>(close_service_name);
    if (!close_client.waitForExistence(
            ros::Duration(close_service_wait_timeout)))
    {
        ROS_FATAL("Laser close service is unavailable");
        return 1;
    }

    if (!ensureLaserClosed(close_client, "starting competition task"))
    {
        return 1;
    }

    int navigation_success_count = 0;
    int shoot_success_count = 0;
    bool safety_failure = false;

    for (std::size_t i = 0; i < points.size() && ros::ok(); ++i)
    {
        const std::size_t number = i + 1;
        ROS_INFO("========== Target %lu/9 ==========",
                 static_cast<unsigned long>(number));

        if (!navigateToPoint(
                move_base_client,
                points[i],
                frame_id,
                number,
                navigation_timeout,
                navigation_retries))
        {
            ROS_ERROR("Skip target %lu because navigation failed",
                      static_cast<unsigned long>(number));
            if (!continue_after_navigation_failure)
            {
                break;
            }
            continue;
        }

        ++navigation_success_count;
        const bool shot_ok =
            runAprilTagShooting(nh, number, shoot_result_param);
        if (shot_ok)
        {
            ++shoot_success_count;
        }

        // 无论射击是否成功，移动前必须再次确认激光关闭。
        if (!ensureLaserClosed(close_client, "moving to next target"))
        {
            ROS_FATAL("Laser state is unsafe; stop the mission");
            safety_failure = true;
            break;
        }

        if (!shot_ok && !continue_after_shoot_failure)
        {
            break;
        }
    }

    bool home_reached = false;
    if (!safety_failure && ros::ok() &&
        ensureLaserClosed(close_client, "returning home"))
    {
        home_reached = navigateToPoint(
            move_base_client,
            home,
            frame_id,
            EXPECTED_TARGET_COUNT + 1,
            navigation_timeout,
            navigation_retries);
    }
    else if (!safety_failure)
    {
        safety_failure = true;
    }

    ROS_INFO(
        "Mission result: navigation=%d/9 shooting=%d/9 home=%s",
        navigation_success_count,
        shoot_success_count,
        home_reached ? "true" : "false");

    if (safety_failure)
    {
        return 2;
    }
    if (!home_reached)
    {
        return 3;
    }
    if (navigation_success_count != 9 || shoot_success_count != 9)
    {
        return 4;
    }
    return 0;
}
