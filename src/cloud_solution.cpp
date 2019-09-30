#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <obstacle_detector/Obstacles.h>

sensor_msgs::PointCloud pointCloud;

void cloudCallback(const obstacle_detector::Obstacles msg)
{
    pointCloud.points.resize(msg.circles.size());
    pointCloud.header.frame_id = "world";
    for (int i = 0; i < msg.circles.size(); i++)
    {
        pointCloud.points[i].x = msg.circles[i].center.x;
        pointCloud.points[i].y = msg.circles[i].center.y;
        pointCloud.points[i].z = msg.circles[i].center.z;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_sol_node");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("raw_obstacles", 100, cloudCallback);
    ros::Publisher pub = node.advertise<sensor_msgs::PointCloud>("cloud_solution",30);

    ros::Rate rate(70.0);
    while (node.ok())
    {
        pub.publish(pointCloud);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};