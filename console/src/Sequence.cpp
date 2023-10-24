#include "rclcpp/rclcpp.hpp"
#include "robot_arm_interface/msg/command.hpp"
#include <vector>
#include <chrono>
#include <thread>

typedef struct {
    uint8_t servo_nr;
    uint16_t pwm;
    uint16_t time_in_ms;
} CommandParameters;

std::string makeCommand(CommandParameters command) {
    return "#" + std::to_string(command.servo_nr) + "P" + std::to_string(command.pwm) + "T" + std::to_string(command.time_in_ms) + "\r";
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    std::vector<CommandParameters> commands {
        {1, 1320, 2000},
        {2, 1390, 2000},
        {3, 1550, 2000},
        {5, 1502, 1000},
        {6, 1502, 1000},
        {1, 1500, 2000},
        {2, 1500, 2000},
        {3, 1500, 2000},
        {5, 1500, 1000},
        {6, 1500, 1000},
    };

    auto node = rclcpp::Node::make_shared("sequence");
    auto publisher = node->create_publisher<robot_arm_interface::msg::Command>("command", 10);


    for (uint8_t i = 0; i <= 2; ++i) {
        auto msg = robot_arm_interface::msg::Command();
        msg.command = makeCommand(commands.at(i));
        rclcpp::spin_some(node);
        publisher->publish(msg);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2100));

    for (uint8_t i = 3; i <= 4; ++i) {
        auto msg = robot_arm_interface::msg::Command();
        msg.command = makeCommand(commands.at(i));
        rclcpp::spin_some(node);
        publisher->publish(msg);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(3100));

    for (uint8_t i = 5; i <= 7; ++i) {
        auto msg = robot_arm_interface::msg::Command();
        msg.command = makeCommand(commands.at(i));
        rclcpp::spin_some(node);
        publisher->publish(msg);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2100));

    for (uint8_t i = 8; i <= 9; ++i) {
        auto msg = robot_arm_interface::msg::Command();
        msg.command = makeCommand(commands.at(i));
        rclcpp::spin_some(node);
        publisher->publish(msg);
    }

    rclcpp::shutdown();

    return 0;
}
