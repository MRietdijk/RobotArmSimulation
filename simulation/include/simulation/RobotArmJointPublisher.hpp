#ifndef ROBOTARMJOINTPUBLISHER_HPP__
#define ROBOTARMJOINTPUBLISHER_HPP__

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "urdf/model.h"

class RobotArmJointPublisher : public rclcpp::Node
{
public:
    RobotArmJointPublisher();
    void initializeUrdf(std::string data);
    void publishJointStates();
    ~RobotArmJointPublisher();
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
    std::unique_ptr<urdf::Model> model;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<std::string> jointNames;
    std::vector<double> jointPositions;
};

#endif // ROBOTARMJOINTPUBLISHER_HPP__