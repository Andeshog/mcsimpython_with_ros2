#include <joystick_interface/joystick_interface.hpp>

JoystickInterfaceNode::JoystickInterfaceNode() : Node("joystick_interface")
{
    set_params();
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickInterfaceNode::joy_callback, this, std::placeholders::_1));
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("joystick/control_input", 10);
}

void JoystickInterfaceNode::set_params()
{
    this->declare_parameter("scale_surge", 100);
    this->declare_parameter("scale_sway", 100);
    this->declare_parameter("scale_yaw", 100);
    scale_surge_ = this->get_parameter("scale_surge").as_int();
    scale_sway_ = this->get_parameter("scale_sway").as_int();
    scale_yaw_ = this->get_parameter("scale_yaw").as_int();
}

void JoystickInterfaceNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    geometry_msgs::msg::Wrench wrench;
    wrench.force.x = msg->axes[1] * scale_surge_;
    wrench.force.y = - msg->axes[0] * scale_sway_;
    wrench.torque.z = msg->axes[3] * scale_yaw_;
    wrench_pub_->publish(wrench);
}