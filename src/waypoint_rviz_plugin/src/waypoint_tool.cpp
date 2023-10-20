#include "waypoint_tool.h"

namespace rviz {
    WaypointTool::WaypointTool() {
        shortcut_key_ = 'w';

        topic_property_ = new StringProperty("Topic", "waypoint", "The topic on which to publish navigation waypionts.",
                                             getPropertyContainer(), SLOT(updateTopic()), this);
    }

    void WaypointTool::onInitialize() {
        PoseTool::onInitialize();
        setName("Waypoint");
        updateTopic();
        vehicle_z = 0;
    }

    void WaypointTool::updateTopic() {
        sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, &WaypointTool::odomHandler, this);
        pub_ = nh_.advertise<geometry_msgs::PointStamped>("/way_point", 5);
        pub_pose = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);
        pub_joy_ = nh_.advertise<sensor_msgs::Joy>("/joy", 5);
    }

    void WaypointTool::odomHandler(const nav_msgs::Odometry::ConstPtr &odom) {
        vehicle_z = odom->pose.pose.position.z;
    }

    void WaypointTool::onPoseSet(double x, double y, double theta) {
        sensor_msgs::Joy joy;

        joy.axes.push_back(0);
        joy.axes.push_back(0);
        joy.axes.push_back(-1.0);
        joy.axes.push_back(0);
        joy.axes.push_back(1.0);
        joy.axes.push_back(1.0);
        joy.axes.push_back(0);
        joy.axes.push_back(0);

        joy.buttons.push_back(0);
        joy.buttons.push_back(0);
        joy.buttons.push_back(0);
        joy.buttons.push_back(0);
        joy.buttons.push_back(0);
        joy.buttons.push_back(0);
        joy.buttons.push_back(0);
        joy.buttons.push_back(1);
        joy.buttons.push_back(0);
        joy.buttons.push_back(0);
        joy.buttons.push_back(0);

        joy.header.stamp = ros::Time::now();
        joy.header.frame_id = "waypoint_tool";
        pub_joy_.publish(joy);

        geometry_msgs::PointStamped waypoint;
        waypoint.header.frame_id = "map";
        waypoint.header.stamp = joy.header.stamp;
        waypoint.point.x = x;
        waypoint.point.y = y;
        waypoint.point.z = vehicle_z;

        // Get the offset between map frame and the planner's frame
        std::string planner_frame = "map"; // Replace with move_base's global frame if different
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);

        ros::Duration timeout(1.0);  // Timeout for transform lookup

        geometry_msgs::TransformStamped transformStamped;

        try {
            transformStamped = tf_buffer.lookupTransform(planner_frame, "map", ros::Time(0), timeout);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }

        // Extract the x and y offsets from the transform
        double x_offset = transformStamped.transform.translation.x;
        double y_offset = transformStamped.transform.translation.y;

        // Create a PoseStamped message
        geometry_msgs::PoseStamped goal;
        goal.pose.position = waypoint.point;
        goal.header.frame_id = planner_frame; // Move_base planner expects goal in the global frame of that planner.
        goal.pose.position.x += x_offset; // Offset by the initial position offset between the two frames
        goal.pose.position.y += y_offset; // Offset by the initial position offset between the two frames
        goal.pose.orientation.w = 1.0; // orientation init

        pub_.publish(waypoint);
        pub_pose.publish(goal);

        usleep(10000);
        pub_.publish(waypoint);
        pub_pose.publish(goal);
    }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rviz::WaypointTool, rviz::Tool
)
