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
    double pwmToRadians(uint16_t pwm);
    void sendJointToPos(uint8_t servo_nr, double pos, uint16_t timeInMs);

    // attributes
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
    rclcpp::Subscription<robot_arm_interface::msg::Command>::SharedPtr commandSub;
    std::unique_ptr<urdf::Model> model;
    rclcpp::TimerBase::SharedPtr timer;
    std::map<std::string, double> joints;

    const std::map<uint8_t, std::string> servoNrToJoints = {
        {0, "base_link2turret"},
        {1, "turret2upperarm"},
        {2, "upperarm2forearm"},
        {3, "forearm2wrist"},
        {4, "wrist2hand"},
        {5, "gripper_left2hand"},
        {6, "gripper_right2hand"}
    };
};

#endif // ROBOTARMJOINTPUBLISHER_HPP__