#include "../include/simulation/CupStatePublisher.hpp"
#include <chrono>

CupStatePublisher::CupStatePublisher() : Node("cup_state_publisher") {
    std::string urdf_xml = this->declare_parameter("robot_description", std::string(""));

    description_pub_ = this->create_publisher<std_msgs::msg::String>(
    "cup_description",
    // Transient local is similar to latching in ROS 1.
    rclcpp::QoS(1).transient_local());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    setupUrdf(urdf_xml);
    
    this->timer = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
        this->publishCup();
    });
}

void CupStatePublisher::setupUrdf(std::string urdf) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = urdf;

    // Publish the robot description
    description_pub_->publish(std::move(msg));
}

void CupStatePublisher::publishCup() {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "cup";

    t.transform.translation.x = 0.45;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.05;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;

    tf_broadcaster_->sendTransform(t);
}

CupStatePublisher::~CupStatePublisher() {

}