#include <joystick_interface/joystick_interface.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Joystick Interface Node");
    auto node = std::make_shared<JoystickInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}