#include "../include/simulation/PickedUpCupListener.hpp"

PickedUpCupListener::PickedUpCupListener(): Node("picked_up_cup_listener")
{
    buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer, this, false);
    publisher = this->create_publisher<geometry_msgs::msg::TransformStamped>("picked_up_cup", 10);
    timer = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        this->timerCallback();
    });
}

void PickedUpCupListener::timerCallback()
{
    geometry_msgs::msg::TransformStamped msg;
    try
    {
        geometry_msgs::msg::TransformStamped transformGripperLeft = buffer->lookupTransform("gripper_left", "cup", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped transformGripperRight = buffer->lookupTransform("gripper_right", "cup", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped transformHand = buffer->lookupTransform("hand", "cup", tf2::TimePointZero);

        if (
            transformGripperLeft.transform.translation.x < -0.006 &&
            transformGripperLeft.transform.translation.x > -0.007 &&
            transformGripperLeft.transform.translation.y < -0.01 &&
            transformGripperLeft.transform.translation.y > -0.015 &&
            transformGripperLeft.transform.translation.z < 0.022 &&
            transformGripperLeft.transform.translation.z > 0.02 &&
            transformGripperRight.transform.translation.x < -0.006 &&
            transformGripperRight.transform.translation.x > -0.007 &&
            transformGripperRight.transform.translation.y < 0.015 &&
            transformGripperRight.transform.translation.y > 0.01 &&
            transformGripperRight.transform.translation.z < 0.022 &&
            transformGripperRight.transform.translation.z > 0.02
        ) {
            msg = transformHand;
        } else {
            msg = buffer->lookupTransform("base_link", "cup", tf2::TimePointZero);
        }
        publisher->publish(msg);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
}

PickedUpCupListener::~PickedUpCupListener()
{
}