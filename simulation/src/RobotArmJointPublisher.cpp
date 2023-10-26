#include "../include/simulation/RobotArmJointPublisher.hpp"
#include "../include/simulation/Parser.hpp"
#include <algorithm>
#include <math.h>
#include <thread>

RobotArmJointPublisher::RobotArmJointPublisher() : Node("robot_arm_joint_publisher") {
    this->declare_parameter("robot_description", std::string(""));
    this->jointStatePub = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states",
        rclcpp::QoS(1)
    );

    this->commandSub = this->create_subscription<robot_arm_interface::msg::Command>(
        "command",
        rclcpp::QoS(10),
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

    Parser p(msg->command);

    if (p.getError()) {
        return;
    } else if (p.getStop()) {
        for (auto &t : movingThreads) {
            pthread_cancel(t.native_handle());
        }
    } else {
        sendJointToPos(p.getServoNr(), pwmToRadians(p.getPwm()), p.getTime());
    }
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

    std::thread t(sendJointFunc, step, servo_nr, pos, timeInMs);

    movingThreads.push_back(std::move(t));
}

double RobotArmJointPublisher::pwmToRadians(uint16_t pwm) {
    pwm = std::clamp(pwm, (uint16_t)1000, (uint16_t)2000);

    return (pwm - 1500) * (M_PI * 2 / 1000);
}

RobotArmJointPublisher::~RobotArmJointPublisher() {

}