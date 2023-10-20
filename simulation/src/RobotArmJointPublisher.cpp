#include "../include/simulation/RobotArmJointPublisher.hpp"
#include <algorithm>
#include <math.h>
#include <thread>

RobotArmJointPublisher::RobotArmJointPublisher() : Node("robot_arm_joint_publisher") {
    // this->descriptionSub = this->create_subscription<std_msgs::msg::String>(
    //     "robot_description",
    //     rclcpp::QoS(1),
    //     [this](const std_msgs::msg::String::SharedPtr msg) {
    //         this->initializeUrdf(msg->data);
    //     }
    // );


    this->declare_parameter("robot_description", std::string(""));
    this->jointStatePub = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states",
        rclcpp::QoS(1)
    );

    this->commandSub = this->create_subscription<robot_arm_interface::msg::Command>(
        "command",
        rclcpp::QoS(1),
        [this](const robot_arm_interface::msg::Command::SharedPtr msg) {
            this->commandCallback(msg);
        }
    );
    
    initializeUrdf(this->get_parameter("robot_description").as_string());

    this->timer = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
        this->publishJointStates();
    });
}

void RobotArmJointPublisher::initializeUrdf(std::string data) {
    urdf::Model model;

    if (!model.initString(data)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
        return;
    }

    for (auto &joint : model.joints_) {
        RCLCPP_INFO(this->get_logger(), "Got joint: %s", joint.first.c_str());

        joints.insert(std::pair<std::string, double>(joint.first, 0));
    }
}

void RobotArmJointPublisher::publishJointStates() {
    if (this->joints.empty()) {
        return;
    }


    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    for (auto const& [jointName, jointPos] : this->joints) {
        msg->name.push_back(jointName);
        msg->position.push_back(jointPos);
    }
    this->jointStatePub->publish(std::move(msg));
}

void RobotArmJointPublisher::commandCallback(const robot_arm_interface::msg::Command::SharedPtr msg) {
    std::cout << "got msg" << std::endl;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got command: %d, %d, %d", msg->servo_nr, msg->pwm, msg->time_in_ms);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pwm: %d equals to %f radians", msg->pwm, pwmToRadians(msg->pwm));

    sendJointToPos(msg->servo_nr, pwmToRadians(msg->pwm), msg->time_in_ms);
}

void RobotArmJointPublisher::sendJointToPos(uint8_t servo_nr, double pos, uint16_t timeInMs) {
    double difference = pos - this->joints.at(this->servoNrToJoints.at(servo_nr));
    double step = 0;
    if (timeInMs != 0) {
        step = difference / timeInMs;
    } else {
        this->joints.at(this->servoNrToJoints.at(servo_nr)) = pos;
        return;
    }
    
    auto sendJointFunc = [this](double step, uint8_t servo_nr, double endPos, uint16_t timeInMs) {
        for (int i = 0; i < timeInMs; i++) {
            this->joints.at(this->servoNrToJoints.at(servo_nr)) += step;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        this->joints.at(this->servoNrToJoints.at(servo_nr)) = endPos;
    };

    std::thread{sendJointFunc, step, servo_nr, pos, timeInMs}.detach();
}

double RobotArmJointPublisher::pwmToRadians(uint16_t pwm) {
    pwm = std::clamp(pwm, (uint16_t)1000, (uint16_t)2000);

    return (pwm - 1500) * (M_PI * 2 / 1000);
}

RobotArmJointPublisher::~RobotArmJointPublisher() {

}