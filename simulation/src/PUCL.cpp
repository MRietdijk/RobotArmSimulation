
#include "../include/simulation/PickedUpCupListener.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PickedUpCupListener>());
    rclcpp::shutdown();
    return 0;
}
