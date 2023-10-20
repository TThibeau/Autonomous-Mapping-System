#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pubPose;

void pointStampedHandler(const geometry_msgs::PointStamped::ConstPtr &waypoint) {
    geometry_msgs::PoseStamped pose_out;

    pose_out.pose.position = waypoint->point;
    pose_out.header.frame_id = "map";
    pose_out.pose.orientation.w = 1.0; // orientation init

    pubPose.publish(pose_out);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "point_to_pose");
    ros::NodeHandle node;

    ros::Subscriber subPoint = node.subscribe<geometry_msgs::PointStamped>("/way_point", 5, pointStampedHandler);

    pubPose = node.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}