#include "rclcpp/rclcpp.hpp"
#include "../include/console/Console.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Console>());
    rclcpp::shutdown();
    return 0;
}
