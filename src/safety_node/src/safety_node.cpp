#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

using std::placeholders::_1;

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// Create ROS subscribers and publishers
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Safety::scan_callback, this, _1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom", 10, std::bind(&Safety::odom_callback, this, _1));
    }

private:
    double speed = 0.0;
    /// create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// Update current speed
        this->speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// calculate TTC
        bool emergency_breaking = false;
        for (std::size_t i=0;i<scan_msg->ranges.size();i++) {
            double r = scan_msg->ranges[i];
            if (std::isnan(r) || r > scan_msg->range_max || r < scan_msg->range_min) continue;
            double threshold = 1; // To be tuned in real vehicle
            if (r / std::max(this->speed * std::cos(scan_msg->angle_min + (double)i * scan_msg->angle_increment), 0.001) < threshold) {
                emergency_breaking = true;
                break;
            }
        }
        // Publish command to brake
        if (emergency_breaking) {
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = 0.0;
            RCLCPP_INFO(this->get_logger(), "emergency brake engaged at speed '%f'", this->speed); // Output to log;
            this->publisher_->publish(drive_msg);
        }
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}