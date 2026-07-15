#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_transformpub");
    ros::NodeHandle nh;
    static tf::TransformBroadcaster transformpub;
    tf::Transform base2laser;
    base2laser.setOrigin(tf::Vector3(1, 0, 0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    base2laser.setRotation(q);
    ros::Rate rate(10);

    while (nh.ok())
    {
        transformpub.sendTransform(tf::StampedTransform(base2laser, ros::Time::now(), "base_link", "laser_link"));
        rate.sleep();
    }

    return 0;
}