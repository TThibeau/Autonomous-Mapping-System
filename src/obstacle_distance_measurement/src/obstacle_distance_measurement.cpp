//
// Created by thibeau on 01.09.23.
//
#include "math.h"
#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

const float minimum_distance = 999;
float minimum_distance_found = minimum_distance;
const float body_buffer_dist = 0.20;

void pointcloudHandler(const sensor_msgs::PointCloud2::ConstPtr &pc) {
    pcl::PointCloud <pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*pc, cloud);

    float distance, dist_x, dist_y, dist_z;
    for (const auto &point: cloud.points) {
        dist_x = abs(point.x);
        dist_y = abs(point.y);
        dist_z = abs(point.z);

        distance = sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z) - body_buffer_dist;

        if (point.z > 0.02 && distance < minimum_distance_found) minimum_distance_found = distance;
//        if (distance < minimum_distance_found) minimum_distance_found = distance;
    }
    printf("Minimum measured since start of test: %0.2f\n", minimum_distance_found);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_distance_measurement");
    ros::NodeHandle nh;

    ros::Subscriber subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 5, pointcloudHandler);

    ros::Rate rate(10);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();

        status = ros::ok();
        rate.sleep();
    }
    return 0;
}