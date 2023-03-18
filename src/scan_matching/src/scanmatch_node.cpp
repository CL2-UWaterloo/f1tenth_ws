#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

#include "scan_matching_skeleton/correspond.h"
#include "scan_matching_skeleton/transform.h"
#include "scan_matching_skeleton/visualization.h"

using namespace std;

const string &TOPIC_SCAN = "/scan";
const string &TOPIC_POS = "/scan_match_location";
const string &TOPIC_RVIZ = "/scan_match_debug";
const string &FRAME_POINTS = "ego_racecar/laser";

const float RANGE_LIMIT = 10.0;

const float MAX_ITER = 2.0;
const float MIN_INFO = 0.1;
const float A = (1 - MIN_INFO) / MAX_ITER / MAX_ITER;

class ScanMatch : public rclcpp::Node
{
    // Implement Scan Matching
    // This is just a template, you are free to implement your own node!

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      TOPIC_SCAN, 1, std::bind(&ScanMatch::handleLaserScan, this, std::placeholders::_1));

    vector<Point> points;
    vector<Point> transformed_points;
    vector<Point> prev_points;
    vector<Correspondence> corresponds;
    vector<vector<int>> jump_table;
    Transform prev_trans, curr_trans;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped tr;

    PointVisualizer *points_viz;
    // CorrespondenceVisualizer* corr_viz;

    geometry_msgs::msg::PoseStamped pose_msg;
    Eigen::Matrix3f global_tf;
    std_msgs::msg::ColorRGBA col;

public:
    ScanMatch() : Node("scanmatch_node")
    {
        pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(TOPIC_POS, 1);
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(TOPIC_RVIZ, 1);
        points_viz = new PointVisualizer(marker_pub, "scan_match", FRAME_POINTS);
        // corr_viz = new CorrespondenceVisualizer(marker_pub, "scan_match", FRAME_POINTS);
        global_tf = Eigen::Matrix3f::Identity(3, 3);
    }

    void handleLaserScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        readScan(msg);

        //We have nothing to compare to!
        if (prev_points.empty())
        {
            RCLCPP_INFO(this->get_logger(), "%s\n", "First Scan");
            prev_points = points;
            return;
        }

        col.r = 1.0;
        col.b = 0.0;
        col.g = 0.0;
        col.a = 1.0;
        points_viz->addPoints(prev_points, col);

        int count = 0;
        computeJump(jump_table, prev_points);
        RCLCPP_INFO(this->get_logger(), "%s\n", "Starting Optimization");

        curr_trans = Transform();

        while (count < MAX_ITER && (curr_trans != prev_trans || count == 0))
        {
            transformPoints(points, curr_trans, transformed_points);

            /// ********* Find correspondence between points of the current and previous frames  *************** ////
            /// getCorrespondence() function is the fast search function and 
            /// getNaiveCorrespondence() function is the naive search option.

            // getCorrespondence(prev_points, transformed_points, points, jump_table, corresponds, A * count * count + MIN_INFO);

            getNaiveCorrespondence(prev_points, transformed_points, points, jump_table, corresponds, A * count * count + MIN_INFO);

            prev_trans = curr_trans;
            ++count;

            /// ********* We update the transforms here ******************************************* ////
            updateTransform(corresponds, curr_trans);
        }

        col.r = 0.0;
        col.b = 0.0;
        col.g = 1.0;
        col.a = 1.0;
        points_viz->addPoints(transformed_points, col);
        points_viz->publishPoints();

        RCLCPP_INFO(this->get_logger(), "Count: %i", count);

        this->global_tf = global_tf * curr_trans.getMatrix();

        publishPos();
        prev_points = points;
    }

    void readScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
    {
        float range_min = msg->range_min;
        float range_max = msg->range_max;
        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;

        const vector<float> &ranges = msg->ranges;
        points.clear();

        for (int i = 0; i < ranges.size(); ++i)
        {
            float range = ranges.at(i);
            if (range > RANGE_LIMIT)
            {
                continue;
            }
            if (!isnan(range) && range >= range_min && range <= range_max)
            {
                points.push_back(Point(range, angle_min + angle_increment * i));
            }
        }
    }

    void publishPos()
    {   
        pose_msg.pose.position.x = global_tf(0, 2);
        pose_msg.pose.position.y = global_tf(1, 2);
        pose_msg.pose.position.z = 0;
        tf2::Matrix3x3 tf3d;
        tf3d.setValue(static_cast<double>(global_tf(0, 0)), static_cast<double>(global_tf(0, 1)), 0,
                    static_cast<double>(global_tf(1, 0)), static_cast<double>(global_tf(1, 1)), 0, 0, 0, 1);

        tf2::Quaternion q;
        tf3d.getRotation(q);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        pose_msg.header.frame_id = FRAME_POINTS;
        pose_msg.header.stamp = this->get_clock()->now();
        pos_pub->publish(pose_msg);

        tr.header.stamp = this->get_clock()->now();
        tr.header.frame_id = "map";
        tr.child_frame_id = "scanmatching_frame";
        tr.transform.translation.x = global_tf(0, 2);
        tr.transform.translation.y = global_tf(1, 2);
        tr.transform.translation.z = 0;

        tr.transform.rotation.x = q.x();
        tr.transform.rotation.y = q.y();
        tr.transform.rotation.z = q.z();
        tr.transform.rotation.w = q.w();
        br->sendTransform(tr);
    }

    ~ScanMatch() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanMatch>());
    rclcpp::shutdown();
    return 0;
}