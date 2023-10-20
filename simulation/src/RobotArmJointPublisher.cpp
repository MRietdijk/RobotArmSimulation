#include "../include/simulation/RobotArmJointPublisher.hpp"

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
        this->jointNames.push_back(joint.first);
        this->jointPositions.push_back(0);
    }
}

void RobotArmJointPublisher::publishJointStates() {
    if (this->jointNames.empty()) {
        return;
    }


    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->header.stamp = this->now();
    msg->name = this->jointNames;
    msg->position = this->jointPositions;

    this->jointStatePub->publish(std::move(msg));
}

void RobotArmJointPublisher::commandCallback(const robot_arm_interface::msg::Command::SharedPtr msg) {
    std::cout << "got msg" << std::endl;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got command: %d, %d, %d", msg->servo_nr, msg->pwm, msg->time_in_ms);
}

RobotArmJointPublisher::~RobotArmJointPublisher() {

}