/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4
 * Pro Flight Stack and tested in Gazebo SITL
 */
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>

Eigen::Vector3d pos_drone;
Eigen::Quaterniond q_drone;

void pos_cb(const nav_msgs::Odometry::ConstPtr& msg) {
    // if (msg->header.frame_id == "t265_odom_frame")
    {
        pos_drone = Eigen::Vector3d(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z);
        // pos_drone_t265[0] = msg->pose.pose.position.x + pos_offset[0];
        // pos_drone_t265[1] = msg->pose.pose.position.y + pos_offset[1];
        // pos_drone_t265[2] = msg->pose.pose.position.z + pos_offset[2];
        q_drone = Eigen::Quaterniond(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        // Euler_t265 = quaternion_to_euler(q_gazebo);
        //  Euler_t265[2] = Euler_t265[2] + yaw_offset;
        //  q_t265 = quaternion_from_rpy(Euler_t265);
    }
    // else
    //{

    //}
}

void pathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    ROS_INFO("Received path with %zu poses.", path_msg->poses.size());
    for (size_t i = 0; i < path_msg->poses.size(); ++i) {
        ROS_INFO("Pose %zu: [%f, %f, %f].", i,
                 path_msg->poses[i].pose.position.x,
                 path_msg->poses[i].pose.position.y,
                 path_msg->poses[i].pose.position.z);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_tf");
    ros::NodeHandle nh;

    //  【订阅】t265估计位置
    ros::Subscriber t265_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 100, pos_cb);

    ros::Subscriber path_sub =
        nh.subscribe("/move_base/DWAPlannerROS/global_plan", 1, pathCallback);

    // ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>(
    //     "/mavros/vision_pose/pose", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    // publish tf

    ros::Rate rate(30.0);
    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(
            tf::Vector3(pos_drone[0], pos_drone[1], pos_drone[2]));
        tf::Quaternion q(q_drone.x(), q_drone.y(), q_drone.z(), q_drone.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                              "odom", "base_link"));
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
