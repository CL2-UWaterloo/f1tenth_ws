#include "waypoint_visualizer.hpp"

#include <math.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

WaypointVisualizer::WaypointVisualizer() : Node("waypoint_visualizer_node") {
    this->declare_parameter("waypoints_path", "/sim_ws/src/pure_pursuit/racelines/e7_floor5.csv");
    this->declare_parameter("rviz_waypoints_topic", "/waypoints");

    waypoints_path = this->get_parameter("waypoints_path").as_string();
    rviz_waypoints_topic = this->get_parameter("rviz_waypoints_topic").as_string();

    vis_path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_waypoints_topic, 1000);
    timer_ = this->create_wall_timer(2000ms, std::bind(&WaypointVisualizer::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "this node has been launched");
    download_waypoints();
}

void WaypointVisualizer::download_waypoints() {  // put all data in vectors
    csvFile_waypoints.open(waypoints_path, std::ios::in);

    RCLCPP_INFO(this->get_logger(), "%s", (csvFile_waypoints.is_open() ? "fileOpened" : "fileNOTopened"));

    // std::vector<std::string> row;
    std::string line, word, temp;

    while (!csvFile_waypoints.eof()) {
        std::getline(csvFile_waypoints, line, '\n');

        std::stringstream s(line);

        int j = 0;
        while (getline(s, word, ',')) {
            RCLCPP_INFO(this->get_logger(), "%s", (word.empty() ? "wordempty" : "wordNOTempty"));
            if (!word.empty()) {
                if (j == 0) {
                    double x = std::stod(word);
                    waypoints.X.push_back(x);
                    RCLCPP_INFO(this->get_logger(), "%f... Xpoint", waypoints.X.back());
                }

                if (j == 1) {
                    waypoints.Y.push_back(std::stod(word));
                    RCLCPP_INFO(this->get_logger(), "%f... Ypoint", waypoints.Y.back());
                }
            }

            j++;
        }
    }

    csvFile_waypoints.close();
}

void WaypointVisualizer::visualize_points() {
    auto marker_array = visualization_msgs::msg::MarkerArray();
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.a = 1.0;
    marker.color.g = 1.0;

    for (unsigned int i = 0; i < waypoints.X.size(); ++i) {
        marker.pose.position.x = waypoints.X[i];
        marker.pose.position.y = waypoints.Y[i];
        marker.id = i;
        marker_array.markers.push_back(marker);
    }

    vis_path_pub->publish(marker_array);
}

void WaypointVisualizer::timer_callback() {
    visualize_points();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<WaypointVisualizer>();  // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
