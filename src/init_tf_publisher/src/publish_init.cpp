#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

double init_x, init_y, init_z = {};

int main(int argc, char **argv) {
    ros::init(argc, argv, "publish_init");
    ros::NodeHandle node;

    node.param<double>("x_init", init_x, 0.0);
    node.param<double>("y_init", init_y, 0.0);
    node.param<double>("z_init", init_z, 0.0);

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;

    // Set the initial position of camera_init relative to odom
    tf::Vector3 map_init_pos(init_x, init_y, init_z);
    tf::Quaternion init_rot(0, 0, 0, 1); // No rotation

    tf::Transform map_transform(init_rot, map_init_pos);

    ros::Rate rate(100.0);

    while (node.ok()) {

        tf::Vector3 camera_init_pos(init_x, init_y, init_z);
        tf::Transform camera_transform(init_rot, camera_init_pos);

        // Publish the transform with respect to odom frame
        broadcaster.sendTransform(tf::StampedTransform(camera_transform, ros::Time::now(), "odom", "camera_init"));
        broadcaster.sendTransform(tf::StampedTransform(map_transform, ros::Time::now(), "odom", "map"));

        rate.sleep();
    }
    return 0;
}