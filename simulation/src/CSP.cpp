#include "../include/simulation/CupStatePublisher.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CupStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
