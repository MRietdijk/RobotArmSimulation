#ifndef ROBOTARMJOINTPUBLISHER_HPP__
#define ROBOTARMJOINTPUBLISHER_HPP__

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "urdf/model.h"
#include "robot_arm_interface/msg/command.hpp"

class RobotArmJointPublisher : public rclcpp::Node
{
public:
    RobotArmJointPublisher();
    void initializeUrdf(std::string data);
    void publishJointStates();
    ~RobotArmJointPublisher();
private:
    void commandCallback(const robot_arm_interface::msg::Command::SharedPtr msg);

    // attributes
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
    rclcpp::Subscription<robot_arm_interface::msg::Command>::SharedPtr commandSub;
    std::unique_ptr<urdf::Model> model;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<std::string> jointNames;
    std::vector<double> jointPositions;
};

#endif // ROBOTARMJOINTPUBLISHER_HPP__