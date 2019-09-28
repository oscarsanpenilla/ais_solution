#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>


float mrk_pose_x;
float mrk_pose_y;
float mrk_pose_z;
float mrk_ori_x;
float mrk_ori_y;
float mrk_ori_z;
float mrk_ori_w;

void poseCallback(const geometry_msgs::PoseStamped msg)
{
    mrk_pose_x = msg.pose.position.x;
    mrk_pose_y = msg.pose.position.y;
    mrk_pose_z = msg.pose.position.z;
    mrk_ori_x = msg.pose.orientation.x;
    mrk_ori_y = msg.pose.orientation.y;
    mrk_ori_z = msg.pose.orientation.z;
    mrk_ori_w = msg.pose.orientation.w;

    // std::cout << msg << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "solution2");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Subscriber vrpn_sub = node.subscribe("vrpn_client_node/BT/pose", 100, poseCallback);

    ros::Rate rate(10.0);
    while (node.ok())
    {
        transform.setOrigin(tf::Vector3(mrk_pose_x, mrk_pose_y, mrk_pose_z));
        transform.setRotation(tf::Quaternion(mrk_ori_x, mrk_ori_y, mrk_ori_z, mrk_ori_w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "img_mrk"));

        transform.setOrigin(tf::Vector3(1.02,0,0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "img_mrk", "base_laser_link"));

        // transform.setOrigin(tf::Vector3(0,0,0));
        // transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_laser_link", "laser"));

        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(3.1415, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_laser_link", "laser"));

        ros::spinOnce();
        rate.sleep();
    }

    
    return 0;
};