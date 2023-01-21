//C++ library includes
#include <memory> //for smart pointers
#include <chrono> //for time 
#include <math.h> //for isnan, isinf etc.
#include <string>
#include <cstdlib> //for abs value function
#include <vector> /// CHECK: might cause errors due to double header in linker
#include <sstream> //for handling csv extraction
//#include <cmath> //may need
#include <iostream> //both for input/output and file stream and handling
#include <fstream>
#include <functional>

//ROS related headers
#include "rclcpp/rclcpp.hpp"
//message header
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>


//headers to code written by you


//other macros
#define NODE_NAME "waypoint_visualiser_node" 
#define _USE_MATH_DEFINES


using std::placeholders::_1;
using namespace std::chrono_literals;


class WaypointVisualiser : public rclcpp::Node {

public:
    WaypointVisualiser() : Node(NODE_NAME) {
        
        vis_path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_waypoints_topic, 1000);
        timer_ = this->create_wall_timer(500ms, std::bind(&WaypointVisualiser::timer_callback, this));

        //initialise tf2 shared pointers
        RCLCPP_INFO (this->get_logger(), "this node has been launched");

        download_waypoints();

        //copy all data once to data structure initialised before

    }

    private:
    struct csvFileData{
        std::vector<double> X;
        std::vector<double> Y;
    };


    //topic names
    std::string rviz_waypoints_topic = "/waypoints";
    
    //file object
    std::fstream csvFile_waypoints; 

    //struct initialisation
    csvFileData waypoints;

    //Publisher initialisation
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_path_pub; 

    //Timer initialisation
    rclcpp::TimerBase::SharedPtr timer_;

    //private functions
    
    void download_waypoints () { //put all data in vectors
        csvFile_waypoints.open("/sim_ws/src/pure_pursuit/src/waypoints_odom.csv", std::ios::in);

        RCLCPP_INFO (this->get_logger(), "%s", (csvFile_waypoints.is_open() ? "fileOpened" : "fileNOTopened"));
        
        //std::vector<std::string> row;
        std::string line, word, temp;

        while (!csvFile_waypoints.eof()) { //TODO: use eof to find end of file (if this doesn't work)
            
            //row.clear(); //empty row extraction string vector

            std::getline(csvFile_waypoints, line, '\n');
            //RCLCPP_INFO (this->get_logger(), "%s", line);


            std::stringstream s(line); //TODO: continue implementation

            int j = 0;
            while (getline(s, word, ',')) {

                RCLCPP_INFO (this->get_logger(), "%s", (word.empty() ? "wordempty" : "wordNOTempty"));
                //RCLCPP_INFO (this->get_logger(), "%f", std::stod(word));
                if (!word.empty()) {

                    //row.push_back(word);

                    if (j == 0) {
                        double x = std::stod(word);
                        waypoints.X.push_back(x);
                        RCLCPP_INFO (this->get_logger(), "%f... Xpoint", waypoints.X.back());
                    }
                    
                    if (j == 1) {
                        waypoints.Y.push_back(std::stod(word));
                        RCLCPP_INFO (this->get_logger(), "%f... Ypoint", waypoints.Y.back());
                    }
                }

                j++;
            }
        }

        csvFile_waypoints.close();
    }

    void visualize_points() {
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

        for(unsigned int i=0; i<waypoints.X.size(); ++i) {
            marker.pose.position.x = waypoints.X[i];
            marker.pose.position.y = waypoints.Y[i];
            marker.id = i;
            marker_array.markers.push_back(marker);
        }

        vis_path_pub->publish(marker_array);
    }

    void timer_callback () {

        visualize_points();
    }
    

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<WaypointVisualiser>(); // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}

