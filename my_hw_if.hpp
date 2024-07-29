#ifndef HW_IF_HPP
#define HW_IF_HPP

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include <vector>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
namespace my_hw_if
{
    class HARDWARE_INTERFACE_PUBLIC MyRobotInterface : public hardware_interface::SystemInterface
    {
        public:
            hardware_interface::HardwareInfo info_;
            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            std::vector<hardware_interface::StateInterface>export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface>export_command_interfaces() override;
            hardware_interface::return_type read(const rclcpp::Time & /*time*/ ,const rclcpp::Duration & /*duration*/) override;
            hardware_interface::return_type write(const rclcpp::Time & /*time*/ ,const rclcpp::Duration & /*duration*/) override;
        private:
            std::vector<double> joint_position;
            std::vector<double> joint_commands;
            std::vector<double> curr_positon;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
            void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
            rclcpp::Node::SharedPtr node_;
    };
}

#endif