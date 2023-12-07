#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
    // Implement Reactive Follow Gap on the car

   public:
    ReactiveFollowGap() : Node("reactive_node") {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->drive_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
    }

   private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    std::vector<double> processed_lidar;

    /// Create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    void preprocess_lidar(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        this->processed_lidar.clear();
        for (int i = 0; i < scan_msg->ranges.size(); i++) {
            if (scan_msg->ranges[i] > 3.0) {
                this->processed_lidar.push_back(0);
            } else {
                this->processed_lidar.push_back(scan_msg->ranges[i]);
            }
        }
    }

    std::pair<int, int> find_max_gap() {
        // Return the start index & end index of the max gap in free_space_ranges
        int largest_starting_i = 0;
        int longest_gap = 0;
        int curr_gap = 0;
        for (int i = 0; i < (int)this->processed_lidar.size(); i++) {
            if (this->processed_lidar[i] < 0.5) {
                curr_gap = 0;
            } else {
                curr_gap++;
                if (curr_gap > longest_gap) {  // Update the largest gap
                    largest_starting_i = i - curr_gap;
                    longest_gap = curr_gap;
                }
            }
        }
        return std::make_pair(largest_starting_i, longest_gap);
    }

    int find_best_point(int starting_i, int gap_distance) {
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
        // Naive: Choose the furthest point within ranges and go there
        int farthest_i = starting_i;
        double farthest_distance = 0;
        for (int i = starting_i; i < starting_i + gap_distance; i++) {
            if (this->processed_lidar[i] > farthest_distance) {
                farthest_i = i;
                farthest_distance = this->processed_lidar[i];
            }
        }
        return farthest_i;
    }

    double to_radians(double theta) {
        return M_PI * theta / 180.0;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR
        // Eliminate all points inside 'bubble' (set them to zero)

        // Find max length gap
        std::pair<int, int> p;
        p = find_max_gap();

        // Find the best point in the gap
        double best_angle_i = find_best_point(p.first, p.second);

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // Fill in drive message and publish
        drive_msg.drive.steering_angle = scan_msg->angle_min + best_angle_i * scan_msg->angle_increment;
        RCLCPP_INFO(this->get_logger(), "a: '%f'", drive_msg.drive.steering_angle);  // Output to log;
        // We go slower if we need to a large steering angle correction
        if (std::abs(drive_msg.drive.steering_angle) >= this->to_radians(0) && std::abs(drive_msg.drive.steering_angle) < this->to_radians(10)) {
            drive_msg.drive.speed = 1.5;
        } else if (std::abs(drive_msg.drive.steering_angle) >= this->to_radians(10) && std::abs(drive_msg.drive.steering_angle) < this->to_radians(20)) {
            drive_msg.drive.speed = 1.0;
        } else {
            drive_msg.drive.speed = 0.5;
        }
        this->publisher_->publish(drive_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
