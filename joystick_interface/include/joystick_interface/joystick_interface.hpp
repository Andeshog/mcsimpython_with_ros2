#ifndef JOYSTICK_INTERFACE_HPP
#define JOYSTICK_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/wrench.hpp>

class JoystickInterfaceNode : public rclcpp::Node
{
    public:
        JoystickInterfaceNode();

    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void set_params();

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub_;
        int scale_surge_;
        int scale_sway_;
        int scale_yaw_;
};

#endif