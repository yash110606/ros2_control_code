#ifndef CUSTOM_CONTROLLER_HPP
#define CUSTOM_CONTROLLER_HPP

#include "hardware_interface/system_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include <vector>
#include <string>
#include "controller_interface/controller_interface_base.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace my_controller
{
    class MyRobotController : public controller_interface::ControllerInterface
    {
        public:
            CONTROLLER_INTERFACE_PUBLIC
            MyRobotController();

            CONTROLLER_INTERFACE_PUBLIC
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;

            CONTROLLER_INTERFACE_PUBLIC
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            CONTROLLER_INTERFACE_PUBLIC
            controller_interface::return_type update(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
            CONTROLLER_INTERFACE_PUBLIC
            
            controller_interface::CallbackReturn on_init() override;

            CONTROLLER_INTERFACE_PUBLIC
            controller_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State & previous_state) override;

            CONTROLLER_INTERFACE_PUBLIC
            controller_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            CONTROLLER_INTERFACE_PUBLIC
            controller_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;
            
            CONTROLLER_INTERFACE_PUBLIC
            controller_interface::CallbackReturn on_cleanup(
                const rclcpp_lifecycle::State & previous_state) override;
            
            CONTROLLER_INTERFACE_PUBLIC
            controller_interface::CallbackReturn on_error(
                const rclcpp_lifecycle::State & previous_state) override;
            
            CONTROLLER_INTERFACE_PUBLIC
            controller_interface::CallbackReturn on_shutdown(
                const rclcpp_lifecycle::State & previous_state) override;
        
        private:
            std::vector<double>ref_val = {1.57,0.5};
            rclcpp::Node::SharedPtr node_;
            std::vector<std::string>joint_names_;
            std::vector<std::string>joint_commands_;
            std::vector<std::string>joint_states_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>command_interfaces;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>state_interfaces;
            std::vector<double>values;
            std::vector<double>cur_output;
    };
}
#endif