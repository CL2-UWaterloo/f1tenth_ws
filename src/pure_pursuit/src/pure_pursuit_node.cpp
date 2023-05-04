/*
Pure Pursuit Implementation in C++. Includes features such as dynamic lookahead. Does not have waypoint 
interpolation yet. 
*/
#include <memory>
#include <math.h>
#include <string>
#include <cstdlib> 
#include <vector> 
#include <sstream> 
#include <iostream> 
#include <fstream>
#include <algorithm> 
#include <Eigen/Eigen>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


#define _USE_MATH_DEFINES
using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node {

public:
    PurePursuit() : Node("pure_pursuit_node") {
        // initialise parameters
        this->declare_parameter("waypoints_path", "/sim_ws/src/pure_pursuit/racelines/e7_floor5.csv");
        this->declare_parameter("odom_topic", "/ego_racecar/odom");
        this->declare_parameter("car_refFrame", "ego_racecar/base_link");
        this->declare_parameter("drive_topic", "/drive");
        this->declare_parameter("rviz_current_waypoint_topic", "/current_waypoint");
        this->declare_parameter("rviz_lookahead_waypoint_topic", "/lookahead_waypoint");
        this->declare_parameter("global_refFrame", "map");
        this->declare_parameter("min_lookahead", 0.5);
        this->declare_parameter("max_lookahead", 1.0);
        this->declare_parameter("lookahead_ratio", 8.0);
        this->declare_parameter("K_p", 0.5);
        this->declare_parameter("steering_limit", 25.0);
        this->declare_parameter("velocity_percentage", 0.6);
        
        // Default Values
        waypoints_path = this->get_parameter("waypoints_path").as_string();
        odom_topic = this->get_parameter("odom_topic").as_string();
        car_refFrame = this->get_parameter("car_refFrame").as_string();
        drive_topic = this->get_parameter("drive_topic").as_string();
        rviz_current_waypoint_topic = this->get_parameter("rviz_current_waypoint_topic").as_string();
        rviz_lookahead_waypoint_topic = this->get_parameter("rviz_lookahead_waypoint_topic").as_string();
        global_refFrame = this->get_parameter("global_refFrame").as_string();
        min_lookahead = this->get_parameter("min_lookahead").as_double();
        max_lookahead = this->get_parameter("max_lookahead").as_double();
        lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
        K_p = this->get_parameter("K_p").as_double();
        steering_limit =  this->get_parameter("steering_limit").as_double();
        velocity_percentage =  this->get_parameter("velocity_percentage").as_double();
        
        //initialise subscriber sharedptr obj
        subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, _1));
        timer_ = this->create_wall_timer(2000ms, std::bind(&PurePursuit::timer_callback, this));

        //initialise publisher sharedptr obj
        publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
        //vis_path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_waypoints_topic, 1000);
        vis_current_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_current_waypoint_topic, 10);
        vis_lookahead_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_lookahead_waypoint_topic, 10);

        //initialise tf2 shared pointers
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO (this->get_logger(), "this node has been launched");

        download_waypoints();

    }

    ~PurePursuit() {}

private:
    //global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    struct csvFileData{
        std::vector<double> X;
        std::vector<double> Y;
        std::vector<double> V;

        int index;
        int velocity_index;

        Eigen::Vector3d lookahead_point_world; // from world reference frame (usually `map`)
        Eigen::Vector3d lookahead_point_car; // from car reference frame
        Eigen::Vector3d current_point_world; // Locks on to the closest waypoint, which gives a velocity profile
    };

    Eigen::Matrix3d rotation_m;
    
    double x_car_world;
    double y_car_world;


    std::string odom_topic;
    std::string car_refFrame;
    std::string drive_topic;
    std::string global_refFrame;
    std::string rviz_current_waypoint_topic;
    std::string rviz_lookahead_waypoint_topic;
    std::string waypoints_path;
    double K_p;
    double min_lookahead;
    double max_lookahead;
    double lookahead_ratio;
    double steering_limit;
    double velocity_percentage;
    double curr_velocity = 0.0;
    
    bool emergency_breaking = false;
    std::string lane_number = "left"; // left or right lane
    
    
    //file object
    std::fstream csvFile_waypoints; 

    //struct initialisation
    csvFileData waypoints;
    int num_waypoints;

    //Timer initialisation
    rclcpp::TimerBase::SharedPtr timer_;

    //declare subscriber sharedpointer obj
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;

    //declare publisher sharedpointer obj
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_drive; 

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_current_point_pub; 
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_lookahead_point_pub; 


    //declare tf shared pointers
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //private functions
    double to_radians(double degrees) {
        double radians;
        return radians = degrees*M_PI/180.0;
    }

    double to_degrees(double radians) {
        double degrees;
        return degrees = radians*180.0/M_PI;
    }
    
    double p2pdist(double &x1, double &x2, double &y1, double &y2) {
        double dist = sqrt(pow((x2-x1),2)+pow((y2-y1),2));
        return dist;
    }

    void download_waypoints () { //put all data in vectors
        csvFile_waypoints.open(waypoints_path, std::ios::in);

        if (!csvFile_waypoints.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path);
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "CSV File Opened");

        }

        
        //std::vector<std::string> row;
        std::string line, word, temp;

        while (!csvFile_waypoints.eof()) {
            std::getline(csvFile_waypoints, line, '\n');
            std::stringstream s(line); 

            int j = 0;
            while (getline(s, word, ',')) {
                if (!word.empty()) {
                    if (j == 0) {
                        waypoints.X.push_back(std::stod(word));
                    } else if (j == 1) {
                        waypoints.Y.push_back(std::stod(word));
                    } else if (j == 2) {
                        waypoints.V.push_back(std::stod(word));
                    }
                }
                j++;
            }
        }

        csvFile_waypoints.close();
        num_waypoints = waypoints.X.size();
        RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %d", num_waypoints, waypoints_path);
        
        double average_dist_between_waypoints = 0.0;
        for (int i=0;i<num_waypoints-1;i++) {
            average_dist_between_waypoints += p2pdist(waypoints.X[i], waypoints.X[i+1], waypoints.Y[i], waypoints.Y[i+1]);
        }
        average_dist_between_waypoints /= num_waypoints;
        RCLCPP_INFO(this->get_logger(), "Average distance between waypoints: %f", average_dist_between_waypoints);
    }

    void visualize_lookahead_point(Eigen::Vector3d &point) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0; 
        marker.color.r = 1.0;

        marker.pose.position.x = point(0);
        marker.pose.position.y = point(1);
        marker.id = 1;
        vis_lookahead_point_pub->publish(marker);
    }

    void visualize_current_point(Eigen::Vector3d &point) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0; 
        marker.color.b = 1.0;

        marker.pose.position.x = point(0);
        marker.pose.position.y = point(1);
        marker.id = 1;
        vis_current_point_pub->publish(marker);
    }

    void get_waypoint() {
        double longest_distance = 0;
        int final_i = -1;
        // Main logic: Search within the next 500 points
        int start = waypoints.index;
        int end = (waypoints.index + 500) % num_waypoints;
        
        // Lookahead needs to be between the min_lookhead and the max_lookahead
        // Set lookahead based on target velocity instead of current velocity? This will provide stronger guarantees.
        // double lookahead = std::min(std::max(min_lookahead, max_lookahead * waypoints.V[waypoints.velocity_index] / lookahead_ratio), max_lookahead); 
        double lookahead = std::min(std::max(min_lookahead, max_lookahead * curr_velocity / lookahead_ratio), max_lookahead); 
        
        
        if (end < start) { // If we need to loop around
            for (int i=start; i<num_waypoints; i++) {
                if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead 
                && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                    longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                    final_i = i;
                }
            }
            for (int i=0; i<end; i++) {
                if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead 
                && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                    longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                    final_i = i;
                }
            }
        } else {
            for (int i=start; i<end; i++) {
                if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead 
                && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                    longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                    final_i = i;
                }
            }

        }

        if (final_i == -1) { // if we haven't found anything, search from the beginning
            final_i = 0; // Temporarily assign to 0
            for (unsigned int i=0;i<waypoints.X.size(); i++) {
                if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= lookahead 
                && p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) >= longest_distance) {
                    longest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                    final_i = i;
                }
            }
        }
        
        // Find the closest point to the car, and use the velocity index for that
        double shortest_distance = p2pdist(waypoints.X[0], x_car_world, waypoints.Y[0], y_car_world);
        int velocity_i = 0;
        for (int i=0; i<waypoints.X.size(); i++) {
            if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= shortest_distance) {
                shortest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
                velocity_i = i;
            }
        }

        // If a waypoint is not found within our radius, then waypoints.index = 0
        waypoints.index = final_i; 
        waypoints.velocity_index = velocity_i;
    }
    
    void quat_to_rot(double q0, double q1, double q2, double q3) { //w,x,y,z -> q0,q1,q2,q3
        double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);
        double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
        double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));
     
        double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
        double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
        double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));
     
        double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
        double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
        double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

        rotation_m << r00, r01, r02, r10, r11, r12, r20, r21, r22; //fill rotation matrix
    }

    void transformandinterp_waypoint() { //pass old waypoint here
        //initialise vectors
        waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;
        // x_car_world, y_car_world should have very similar values
        waypoints.current_point_world << waypoints.X[waypoints.velocity_index], waypoints.Y[waypoints.velocity_index], 0.0; 

        //rviz publish way point
        visualize_lookahead_point(waypoints.lookahead_point_world);
        visualize_current_point(waypoints.current_point_world);
        //look up transformation at that instant from tf_buffer_
        geometry_msgs::msg::TransformStamped transformStamped;
        
        try {
            // Get the transform from the base_link reference to world reference frame
            transformStamped = tf_buffer_->lookupTransform(car_refFrame, global_refFrame,tf2::TimePointZero);
        } 
        catch (tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform. Error: %s", ex.what());
        }

        //transform points (rotate first and then translate)
        Eigen::Vector3d translation_v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

        waypoints.lookahead_point_car = (rotation_m*waypoints.lookahead_point_world) + translation_v;
    }

    double p_controller() { // pass waypoint
        double r = waypoints.lookahead_point_car.norm(); // r = sqrt(x^2 + y^2)
        double y = waypoints.lookahead_point_car(1);
        double angle = K_p * 2 * y / pow(r, 2); // Calculated from https://docs.google.com/presentation/d/1jpnlQ7ysygTPCi8dmyZjooqzxNXWqMgO31ZhcOlKVOE/edit#slide=id.g63d5f5680f_0_33

        return angle;
    }

    double get_velocity(double steering_angle) {
        double velocity;
        
        if (waypoints.V[waypoints.velocity_index]) {
            velocity = waypoints.V[waypoints.velocity_index] * velocity_percentage;
        } else { // For waypoints loaded without velocity profiles
            if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
                velocity = 6.0 * velocity_percentage; 
            } 
            else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
                velocity = 2.5 * velocity_percentage;
            } 
            else {
                velocity = 2.0 * velocity_percentage;
            }
        }

        // if (emergency_breaking) velocity = 0.0;  // Do not move if you are about to run into a wall
            
        return velocity;
    }

    void publish_message (double steering_angle) {
        auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();
        if (steering_angle < 0.0) {
            drive_msgObj.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit)); //ensure steering angle is dynamically capable
        } else {
            drive_msgObj.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit)); //ensure steering angle is dynamically capable
        }

        curr_velocity = get_velocity(drive_msgObj.drive.steering_angle);
        drive_msgObj.drive.speed = curr_velocity;

        RCLCPP_INFO(this->get_logger(), "index: %d ... distance: %.2fm ... Speed: %.2fm/s ... Steering Angle: %.2f ... K_p: %.2f ... velocity_percentage: %.2f", waypoints.index, p2pdist(waypoints.X[waypoints.index], x_car_world, waypoints.Y[waypoints.index], y_car_world), drive_msgObj.drive.speed, to_degrees(drive_msgObj.drive.steering_angle), K_p, velocity_percentage);

        publisher_drive->publish(drive_msgObj);
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
        x_car_world = odom_submsgObj->pose.pose.position.x;
        y_car_world = odom_submsgObj->pose.pose.position.y;
        // interpolate between different way-points 
        get_waypoint();

        //use tf2 transform the goal point 
        transformandinterp_waypoint();

        // Calculate curvature/steering angle
        double steering_angle = p_controller();

        //publish object and message: AckermannDriveStamped on drive topic 
        publish_message(steering_angle);
    }
    
    void timer_callback () {
        // Periodically check parameters and update
        K_p = this->get_parameter("K_p").as_double();
        velocity_percentage = this->get_parameter("velocity_percentage").as_double();
        min_lookahead = this->get_parameter("min_lookahead").as_double();
        max_lookahead = this->get_parameter("max_lookahead").as_double();
        lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
        steering_limit =  this->get_parameter("steering_limit").as_double();
    }
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}