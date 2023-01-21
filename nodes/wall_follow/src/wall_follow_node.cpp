#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

using std::placeholders::_1;


class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // Create ROS subscribers and publishers
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(this->drive_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, _1));
    }

private:
    // PID CONTROL PARAMS
    double kp = 3;
    double ki = 0;
    double kd = 0.1;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double start_t = -1;
    double curr_t = 0.0;
    double prev_t = 0.0;
    
    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// Create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    

    double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR in radians

        Returns:
            range: range measurement in meters at the given angle
        */

        assert(angle >= scan_msg->angle_min && angle <= scan_msg->angle_max); // Angle must be within range
        int i = (angle - scan_msg->angle_min) / (scan_msg->angle_increment); // index i of closest angle
        if (std::isnan(scan_msg->ranges[i]) || scan_msg->ranges[i] > scan_msg->range_max) return scan_msg->range_max; // In case of NaNs and infinity, just return the maximum of the scan message
        return scan_msg->ranges[i];
    }
    
    double to_radians(double theta) {
        return M_PI * theta / 180.0;

    }
    
    double to_degrees(double theta) {
        return theta * 180.0 / M_PI;
    }

    void get_error(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        double a = get_range(scan_msg, to_radians(-50.0));
        double b = get_range(scan_msg, to_radians(-90.0)); // 0 degrees is in front of the card.
        double theta = to_radians(40.0); // 90.0 - 50.0 = 40.0 degrees
        double alpha = std::atan((a * std::cos(theta) - b)/(a * std::sin(theta)));
        double D_t = b*std::cos(alpha);
        // double D_t_1 =  D_t + dist * std::sin(alpha);

        this->prev_error = this->error;
        this->error = dist - D_t;
        this->integral += this->error;
        this->prev_t = this->curr_t;
        this->curr_t = (double) scan_msg->header.stamp.nanosec * (double)10e-9 + (double) scan_msg->header.stamp.sec;
        if (this->start_t == 0.0) {
            this->start_t = this->curr_t;
        }
    }

    void pid_control()
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;
        // Use kp, ki & kd to implement a PID controller
        if (this->prev_t == 0.0) return;
        angle = this->kp * this->error + this->ki * this->integral * (this->curr_t - this->start_t) + this->kd * (this->error - this->prev_error)/(this->curr_t - this->prev_t);

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // Fill in drive message and publish
        drive_msg.drive.steering_angle = angle;
        
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

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        get_error(scan_msg, 1); 
        pid_control();

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}