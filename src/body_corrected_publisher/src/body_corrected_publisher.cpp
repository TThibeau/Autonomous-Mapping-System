//
// Created by thibeau on 31.08.23.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

float sensor_height = 0.128;

int main(int argc, char** argv) {
    ros::init(argc, argv, "body_corrected_publisher");
    ros::NodeHandle node;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;

    std::string target_frame = "map";    // The target frame for publishing the corrected transform
    std::string corrected_frame = "body_corrected"; // The corrected frame name
    std::string original_frame = "lio_sam_sensor"; // The original frame to correct

    ros::Rate rate(10.0); // Publish rate in Hz

    while (node.ok()) {
        tf::StampedTransform original_transform;

        try {
            // Listen for the transform between "map" and "body"
            listener.lookupTransform(target_frame, original_frame, ros::Time(0), original_transform);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
        }

        // Correct the translation and rotation
        tf::Vector3 corrected_translation(original_transform.getOrigin().getX(),
                                          original_transform.getOrigin().getY(),
                                          sensor_height); // Set z-component to sensor_height
        tf::Quaternion corrected_rotation;
        corrected_rotation.setRPY(0.0, 0.0, tf::getYaw(original_transform.getRotation())); // Reset roll and pitch

        // Create the corrected transform
        tf::Transform corrected_transform(corrected_rotation, corrected_translation);

        // Broadcast the corrected transform as "body_corrected"
        broadcaster.sendTransform(tf::StampedTransform(corrected_transform, ros::Time::now(), target_frame, corrected_frame));

        rate.sleep();
    }

    return 0;
}