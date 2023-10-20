#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_offset_fixer");
    ros::NodeHandle node;

    // Create a publisher to publish the point cloud
    ros::Publisher pubPointCloud = node.advertise<sensor_msgs::PointCloud2>("velodyne_points_corrected", 1);

    // Set the header's frame ID to the existing frame
    std::string existing_frame = "body_corrected"; // Replace with the desired frame name

    // Create a subscriber to the point cloud topic
    ros::Subscriber subPointCloud = node.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1,
                                                                             [&](const sensor_msgs::PointCloud2::ConstPtr &input_cloud) {
                                                                                 // Create a point cloud message with the existing frame ID
                                                                                 sensor_msgs::PointCloud2 point_cloud_msg = *input_cloud;
                                                                                 point_cloud_msg.header.frame_id = existing_frame;

                                                                                 // Publish the point cloud with the modified frame ID
                                                                                 pubPointCloud.publish(point_cloud_msg);
                                                                             });

    ros::spin();

    return 0;
}