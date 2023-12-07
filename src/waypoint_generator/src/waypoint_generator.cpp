#include "waypoint_generator.hpp"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

WaypointGenerator::WaypointGenerator() : Node("waypoint_generator_node") {
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("min_distance", 0.05);
    this->declare_parameter("save_path", "/sim_ws/src/waypoint_generator/src/waypoints_odom.csv");

    odom_topic = this->get_parameter("odom_topic").as_string();
    min_distance = this->get_parameter("min_distance").as_double();
    save_path = this->get_parameter("save_path").as_string();

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 1000, std::bind(&WaypointGenerator::odom_callback, this, std::placeholders::_1));
}

void WaypointGenerator::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
    csv_odom.open(save_path, std::ios::out | std::ios::app);

    double diff = sqrt(pow((odom_submsgObj->pose.pose.position.x - x_old), 2) + pow((odom_submsgObj->pose.pose.position.y - y_old), 2));

    if (diff > min_distance) {
        double x = odom_submsgObj->pose.pose.position.x;
        double y = odom_submsgObj->pose.pose.position.y;
        csv_odom << "\n"
                 << x << ", " << y;

        RCLCPP_INFO(this->get_logger(), "%f....%f", odom_submsgObj->pose.pose.position.x, odom_submsgObj->pose.pose.position.y);
        RCLCPP_INFO(this->get_logger(), "%f", diff);

        x_old = odom_submsgObj->pose.pose.position.x;
        y_old = odom_submsgObj->pose.pose.position.y;
    }

    csv_odom.close();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<WaypointGenerator>();  // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
