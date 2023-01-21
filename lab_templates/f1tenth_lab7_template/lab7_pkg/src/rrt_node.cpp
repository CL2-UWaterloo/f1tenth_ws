#include "rrt/rrt.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRT>());
    rclcpp::shutdown();
    return 0;
}