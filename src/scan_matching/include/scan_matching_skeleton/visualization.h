#pragma once

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "scan_matching_skeleton/correspond.h"

using namespace std;

static int num_visuals = 0;

class PointVisualizer {
protected:
	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub;
	visualization_msgs::msg::Marker dots;
	rclcpp::Clock::SharedPtr clock;
	string ns;
	string frame_id;

public:
	PointVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub, string ns, string frame_id);
	void addPoints(vector<Point>& points, std_msgs::msg::ColorRGBA color);
	void publishPoints();
    ~PointVisualizer() {};
};

// class CorrespondenceVisualizer {
// protected:
// 	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub;
// 	visualization_msgs::msg::Marker line_list;
// 	string ns;
// 	string frame_id;

// public:
// 	CorrespondenceVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub, string ns, string frame_id);
// 	void addCorrespondences(vector<Correspondence> corresponds);
// 	void publishCorrespondences();
//     ~CorrespondenceVisualizer() {};
// };
