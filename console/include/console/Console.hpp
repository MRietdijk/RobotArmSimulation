#ifndef CONSOLE_HPP__
#define CONSOLE_HPP__

#include "rclcpp/rclcpp.hpp"
#include <robot_arm_interface/msg/command.hpp>

class Console : public rclcpp::Node
{
public:
    Console();
    void askForCommand();
    ~Console();
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<robot_arm_interface::msg::Command>::SharedPtr commandPub;
};


#endif // CONSOLE_HPP__