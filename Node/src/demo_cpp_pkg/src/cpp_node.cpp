#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <string>


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("love_it");
    RCLCPP_INFO(node->get_logger(), "Hello World!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}