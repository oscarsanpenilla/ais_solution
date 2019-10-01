#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <obstacle_detector/Obstacles.h>
#include <math.h>
#include <geometry_msgs/Point32.h>

sensor_msgs::PointCloud pointCloud;
sensor_msgs::PointCloud pointCloudVel;

void cloudCallback(const obstacle_detector::Obstacles msg)
{
    pointCloud.points.resize(msg.circles.size());
    pointCloud.header.frame_id = "world";
    pointCloud.header.stamp = msg.header.stamp;
    pointCloudVel.header.frame_id = "world";
    pointCloudVel.header.stamp = msg.header.stamp;
    pointCloudVel.points.resize(0);
    float vel   = 0;
    float vel_x = 0;
    float vel_y = 0;
    for (int i = 0; i < msg.circles.size(); i++)
    {
        pointCloud.points[i].x = msg.circles[i].center.x;
        pointCloud.points[i].y = msg.circles[i].center.y;
        pointCloud.points[i].z = msg.circles[i].center.z;

        vel_x = msg.circles[i].velocity.x;
        vel_y = msg.circles[i].velocity.y;
        vel = sqrt(vel_x*vel_x + vel_y*vel_y);

        if (vel >= 0.26 && vel <= 0.265)
        {
            pointCloudVel.points.push_back(pointCloud.points[i]);
        }
        
        
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_sol_node");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("obstacles", 100, cloudCallback);
    ros::Publisher pub = node.advertise<sensor_msgs::PointCloud>("cloud_solution",30);
    ros::Publisher pub_move = node.advertise<sensor_msgs::PointCloud>("cloud_obj_movement",30);

    ros::Rate rate(70.0);
    while (node.ok())
    {
        pub.publish(pointCloud);
        pub_move.publish(pointCloudVel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
};