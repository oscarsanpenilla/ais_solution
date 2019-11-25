#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>


sensor_msgs::LaserScan laserScanTrans;
sensor_msgs::PointCloud laserToPointCloud;
sensor_msgs::PointCloud laserPointCloud;

void laserCallback(const sensor_msgs::LaserScan msg)
{
    sensor_msgs::LaserScan laserScan = msg;
    laserPointCloud.points.resize(laserScan.ranges.size());
    for (int i = 0; i < laserScan.ranges.size(); i++)
    {
        float angle = laserScan.angle_min + i * laserScan.angle_increment;
        float radio = laserScan.ranges[i];
        laserPointCloud.points[i].x = radio * cos(angle);
        laserPointCloud.points[i].y = radio * sin(angle);
        laserPointCloud.points[i].z = 0;
    }
    laserPointCloud.header.frame_id = "world";
    laserPointCloud.header.stamp = ros::Time::now();

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_transformer");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;
    tf::Transform transform;

    ros::Subscriber scan_sub = node.subscribe("scan", 100, laserCallback);
    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud>("transformed_scan", 50);

    ros::Rate rate(30.0);
    while (node.ok())
    {

        try
        {
            listener.waitForTransform("world", "laser", ros::Time::now(), ros::Duration(5));
            listener.transformPointCloud("world",laserPointCloud,laserToPointCloud);
            laserToPointCloud.points.resize(laserPointCloud.points.size());
            // for (int i = 0; i < laserPointCloud.points.size(); i++)
            // {
            //     float angle = atan2(laserPointCloud.points[i].y,laserPointCloud.points[i].x);
            //     float radio = laserPointCloud.points[i].x / cos(angle);
            //     laserScanTrans.ranges[i] = radio;
            // }
            // laserScanTrans.header.frame_id = "/world";
            // laserScanTrans.header.stamp = ros::Time::now();
            // laserScanTrans.angle_min = -1.5708758831;
            // laserScanTrans.angle_max = 1.57040476799;
            // laserScanTrans.angle_increment = 0.00581718236208;
            // laserScanTrans.time_increment = 6.17283949396e-05;

            cloud_pub.publish(laserToPointCloud);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};