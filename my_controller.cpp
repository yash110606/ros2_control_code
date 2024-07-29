#include "my_controller/my_controller.hpp"
#include "iostream"
#include <stddef.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using config_type=controller_interface::interface_configuration_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace my_controller
{
    MyRobotController::MyRobotController() : controller_interface::ControllerInterface(){
    }
    controller_interface::CallbackReturn MyRobotController::on_init()
    {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
        joint_commands_ = auto_declare<std::vector<std::string>>("command_interfaces",joint_commands_);
        joint_states_ = auto_declare<std::vector<std::string>>("state_interfaces",joint_states_);
        auto name = joint_names_[1];
        RCLCPP_INFO(get_node()->get_logger(),"size of joint_names_  : %li",joint_names_.size());
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::InterfaceConfiguration MyRobotController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type=controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names={"base_arm_joint/position","part1_part2_joint/position"};
        return config;
    }
    controller_interface::InterfaceConfiguration MyRobotController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type=controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names={"base_arm_joint/position","part1_part2_joint/position"};
        RCLCPP_INFO(get_node()->get_logger(),"size of command_interface_config names : %li",config.names.size());
        return config;
    }
    controller_interface::return_type MyRobotController::update(const rclcpp::Time & time,const rclcpp::Duration & duration)
    {
        for(size_t i=0;i<state_interfaces.size();i++)
        {
            double val = state_interfaces[i].get().get_value();
            RCLCPP_INFO(get_node()->get_logger(),"%s position value : %f ",state_interfaces[i].get().get_name().c_str(),state_interfaces[i].get().get_value());
            if(val<ref_val[i])
            {
                command_interfaces[i].get().set_value(val+0.001);
            }
        }
        return controller_interface::return_type::OK;
    }
    controller_interface::CallbackReturn MyRobotController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(get_node()->get_logger(),"initiating configuration sequence ");
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn MyRobotController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        RCLCPP_INFO(get_node()->get_logger(),"initiating activation sequence");
        RCLCPP_INFO(get_node()->get_logger(),"size of state_interfaces_ : %li",state_interfaces_.size());
        for(size_t i=0;i<state_interfaces_.size();i++)
        {
            state_interfaces.push_back(state_interfaces_[i]);
        }
        RCLCPP_INFO(get_node()->get_logger(),"size of loaned interface : %li",state_interfaces_.size());
        for(size_t i=0;i<command_interfaces_.size();i++)
        {
            command_interfaces.push_back(command_interfaces_[i]);
        }
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn MyRobotController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn MyRobotController::on_cleanup(const rclcpp_lifecycle::State & previous_state)
    {
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn MyRobotController::on_error(const rclcpp_lifecycle::State & previous_state)
    {
        return CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn MyRobotController::on_shutdown(const rclcpp_lifecycle::State & previous_state)
    {
        return CallbackReturn::SUCCESS;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    my_controller::MyRobotController,controller_interface::ControllerInterface
)