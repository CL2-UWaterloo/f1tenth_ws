//C++ library includes
#include <memory> //for smart pointers
//#include <chrono> //for time 
//#include <math.h> //for isnan, isinf etc.
#include <string>
#include <cstdlib> //for abs value function
//#include <vector> /// CHECK: might cause errors due to double header in linker
#include <iostream>
#include <fstream>


//ROS related headers
#include "rclcpp/rclcpp.hpp"
//message header
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


//headers to code written by you


//other macros
#define NODE_NAME "waypoint_subscriber_node" 
//#define X_DIST 0.2
//#define Y_DIST 0.2
#define DIST 0.05

//using namespaces 
//used for bind (uncomment below)
using std::placeholders::_1;
//using namespace std::chrono_literals;


class WaypointSubscriber : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    WaypointSubscriber() : Node(NODE_NAME) {
        //initialise subscriber sharedptr obj
        subscription_point = this->create_subscription<geometry_msgs::msg::PoseStamped>(point_topic, 1000, std::bind(&WaypointSubscriber::point_callback, this, _1));
        subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 1000, std::bind(&WaypointSubscriber::odom_callback, this, _1));

        publisher_rviz_point = this->create_publisher<geometry_msgs::msg::PoseStamped>(point_topic_rviz, 1000);
        publisher_rviz_odom = this->create_publisher<geometry_msgs::msg::PoseStamped>(odom_topic_rviz, 1000);

        RCLCPP_INFO (this->get_logger(), "this node has been launched");

    }

private:

    //global static (to be shared by all objects) and dynamic variables (each instance gets its own copy -> managed on the stack)
    //std::string lidarscan_topic = "/scan";
    std::string odom_topic = "/ego_racecar/odom";
    std::string point_topic = "/clicked_point";
    std::string point_topic_rviz = "/waypoint_point_rviz";
    std::string odom_topic_rviz = "/waypoint_odom_rviz";
    //std::string rviz_debug_topic = "/rviz_custom_debugger";    
    //declare publisher sharedpointer obj
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_point;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_rviz_point;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_rviz_odom;

    //file initialisation
    std::ofstream csvFile_point; 
    std::ofstream csvFile_odom;
    std::string comma = ",";
    std::string newline = "\n";

    double x_old = 0;
    double y_old = 0;    

    void point_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr point_submsgObj) {   
        //write to file here --> might have to subscribe to publish point instead of odometry to get point in global frame
        csvFile_point.open("waypoints_point.csv", std::ios::app);

        csvFile_point << point_submsgObj->pose.position.x;
        csvFile_point <<',';
        csvFile_point << point_submsgObj->pose.position.y;
        csvFile_point <<"\n";

        csvFile_point.close();

        auto rviz_point_msgObj = geometry_msgs::msg::PoseStamped();
        rviz_point_msgObj.pose.position.x = point_submsgObj->pose.position.x;
        rviz_point_msgObj.pose.position.y = point_submsgObj->pose.position.y;

        publisher_rviz_point->publish(rviz_point_msgObj);

        RCLCPP_INFO (this->get_logger(), "%f....%f", point_submsgObj->pose.position.x, point_submsgObj->pose.position.y);


    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {   
        csvFile_odom.open("/sim_ws/src/waypoint_generator/src/waypoints_odom.csv", std::ios::out | std::ios::app);

        double diff = sqrt(pow((odom_submsgObj->pose.pose.position.x-x_old),2)+pow((odom_submsgObj->pose.pose.position.y-y_old),2));

        if( diff > DIST) {
            double x = odom_submsgObj->pose.pose.position.x;
            double y = odom_submsgObj->pose.pose.position.y;
            csvFile_odom << "\n" << x << ", " << y;

            RCLCPP_INFO (this->get_logger(), "%f....%f", odom_submsgObj->pose.pose.position.x, odom_submsgObj->pose.pose.position.y);
            RCLCPP_INFO (this->get_logger(), "%f", diff);

            
            x_old = odom_submsgObj->pose.pose.position.x;
            y_old = odom_submsgObj->pose.pose.position.y;

            auto rviz_point_msgObj = geometry_msgs::msg::PoseStamped(); // TODO: change to visualisation marker
            rviz_point_msgObj.pose.position.x = odom_submsgObj->pose.pose.position.x;
            rviz_point_msgObj.pose.position.y = odom_submsgObj->pose.pose.position.y;

            publisher_rviz_odom->publish(rviz_point_msgObj);

        }

        csvFile_odom.close();

    }

};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<WaypointSubscriber>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}


/*
    what other data do I need? --> like heading angle, speed etc. 
    write scripts to try better waypoints (e.g. spline, smoothing etc.)
*/

