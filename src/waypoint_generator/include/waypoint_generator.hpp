#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class WaypointGenerator : public rclcpp::Node {
   public:
    WaypointGenerator();

   private:
    std::string odom_topic;
    double min_distance;
    std::string save_path;

    double x_old = 0;
    double y_old = 0;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_point;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_rviz_point;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_rviz_odom;

    std::ofstream csv_odom;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj);
};