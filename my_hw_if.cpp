#include "my_hw_if/my_hw_if.hpp"

namespace my_hw_if
{
    CallbackReturn MyRobotInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        info_=info;
        joint_commands.resize(info.joints.size(),0);
        joint_position.resize(info.joints.size(),0);
        curr_positon.resize(info.joints.size(),0);
        node_ = std::make_shared<rclcpp::Node>("my_hw_if");
        publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
        RCLCPP_INFO(node_->get_logger(),"%li",info.joints.size());
        return CallbackReturn::SUCCESS;
    }
    std::vector<hardware_interface::StateInterface> MyRobotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface>state_interfaces;
        for(size_t i=0;i<info_.joints.size();i++)
        {
            state_interfaces.emplace_back(info_.joints[i].name,"position",&joint_position[i]);
        }
        RCLCPP_INFO(node_->get_logger(),"size of the state_interfaces : %li",state_interfaces.size());
        return state_interfaces;
    }
    std::vector<hardware_interface::CommandInterface> MyRobotInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface>command_interfaces;
        for(size_t i=0;i<info_.joints.size();i++)
        {
            command_interfaces.emplace_back(info_.joints[i].name,"position",&joint_commands[i]);
            RCLCPP_INFO(node_->get_logger(),"name : %s , interface_name : %s , value : %f ",command_interfaces[i].get_name().c_str(),command_interfaces[i].get_interface_name().c_str(),command_interfaces[i].get_value());
        }
        RCLCPP_INFO(node_->get_logger(),"exporting Command Interface");
        return command_interfaces;
    } 
    void MyRobotInterface::joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if(msg->name.size()!=0)
        {
            for(size_t i=0;i<msg->name.size();i++)
            {
                curr_positon[i]=msg->position[i];
                RCLCPP_INFO(node_->get_logger(),"%s position value : %f",msg->name[i].c_str(),msg->position[i]);
            }
        }
    }
    hardware_interface::return_type MyRobotInterface::write(const rclcpp::Time & time,const rclcpp::Duration & duration)
    {
        auto mes = sensor_msgs::msg::JointState();
        mes.header.stamp = node_->get_clock()->now();
        mes.name={"base_arm_joint","part1_part2_joint"};
        for(size_t i=0;i<info_.joints.size();i++)
        {
            mes.position.emplace_back(joint_commands[i]);
        }
        mes.velocity={1.0,1.0};
        mes.effort={1000,1000};
        publisher_->publish(mes);
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type MyRobotInterface::read(const rclcpp::Time & time ,const rclcpp::Duration & duration)
    {
        subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,std::bind(&MyRobotInterface::joint_state_callback,this,std::placeholders::_1));
        for(size_t i=0;i<info_.joints.size();i++)
        {
            joint_position[i]=curr_positon[i];
        }
        return hardware_interface::return_type::OK;       
    }
}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_hw_if::MyRobotInterface, hardware_interface::SystemInterface)
