#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <vector>

float pot_pose_x;
float pot_pose_y;
float pot_pose_z;
sensor_msgs::PointCloud pointCloud;

void cloudCallback(const sensor_msgs::PointCloud msg)
{
    pointCloud = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_transformer");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::TransformListener listener;

    ros::Subscriber cloud_sub = node.subscribe("pot_cloud", 100, cloudCallback);
    ros::Publisher cloud_pub = node.advertise<sensor_msgs::PointCloud>("transformed_cloud", 50);

    ros::Rate rate(10.0);
    while (node.ok())
    {

        sensor_msgs::PointCloud pointCloudTrans;
        try
        {
            listener.waitForTransform("world", "base_laser_link", ros::Time::now(), ros::Duration(0.5));
            listener.transformPointCloud("world", pointCloud, pointCloudTrans);
            cloud_pub.publish(pointCloudTrans);
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