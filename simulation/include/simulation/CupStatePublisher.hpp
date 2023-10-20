#ifndef CUPSTATEPUBLISHER_HPP__
#define CUPSTATEPUBLISHER_HPP__

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <map>
#include <std_msgs/msg/string.hpp>

class CupStatePublisher : public rclcpp::Node
{
public:
    CupStatePublisher();
    void setupUrdf(std::string urdf);
    void publishCup();
    ~CupStatePublisher();
private:
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer;

    /// A pointer to the ROS 2 publisher for the robot_description
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;
};


#endif // CUPSTATEPUBLISHER_HPP__