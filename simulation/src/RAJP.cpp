#include "../include/simulation/RobotArmJointPublisher.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotArmJointPublisher>());
    rclcpp::shutdown();
    return 0;
}
