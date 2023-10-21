#ifndef PICKEDUPCUPLISTENER_HPP__
#define PICKEDUPCUPLISTENER_HPP__

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


class PickedUpCupListener : public rclcpp::Node
{
public:
    PickedUpCupListener();
    ~PickedUpCupListener();
private:
    void timerCallback();

    //attributes
    std::shared_ptr<tf2_ros::TransformListener> listener;
    std::shared_ptr<tf2_ros::Buffer> buffer;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
};


#endif