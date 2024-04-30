#include <controller_interface/controller_interface.hpp>
#include <rclcpp.lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "chienpanze_controller_itnerface/chienpanze_controller_interface.hpp"

#include <vector>
#include <string>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;[]()

controller_interface::CallbackReturn on_init(){
    // declare and get parameters needed for controller initialization
    // allocate memory that will exist for the life of the controller

    // set positions at zero
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state){
    // declare and get parameters needed for controller operations
    // setup realtime buffers, ROS publishers, and ROS subscribers
    // ...
    //auto result = get_command_interface();

    command_interface_types_.clear();
    for(const std::string& joint_name : joint_names_){
        std::string interface_name = joint_name + "/position";

    }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration command_interface_configuration(){
    controller_interface::InterfaceConfiguration conf;
    // add required command interface to `conf` by specifying their names and interface types.
    // ..
    conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for(const auto& name: joint_names){
        conf.names.push_back(name + "/position");
    }

    return conf
}

controller_interface::InterfaceConfiguration state_interface_configuration() {
    controller_interface::InterfaceConfiguration conf;
    // add required state interface to `conf` by specifying their names and interface types.
    // ..
    return conf
}

controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state){
  // Handle controller restarts and dynamic parameter updating
  // ...

    // clear out vectors in case of restart
//   joint_position_command_interface_.clear();
//   joint_velocity_command_interface_.clear();
//   joint_position_state_interface_.clear();
//   joint_velocity_state_interface_.clear();

  return CallbackReturn::SUCCESS;
}

std::vector<double> SimpleController::get_current_positions(){
    std::vector<double> positions = {};
    return positions;
}

std::vector<double> SImpleController::compute_position_commands(const std::vector<double>& current_positions){
    std:vector<double> commands = {};
    for(auto& pos: current_positions){
        double target = 0.0;
        double error = target - pos;
        commands.push_back(error);
    }
    return commands;
}

void set_command(size_t& joint_index, double& command_value){
    if(joint_index < command_interface_types_.size()){
        command_interface_types_[joint_index].set_value(command_value);
    }
}


controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period){
  // Read controller inputs values from state interfaces
  // Calculate controller output values and write them to command interfaces
    std::vector<double> current_positions = get_current_positions();
    std::vector<double> commands = compute_position_commands(current_positions);

    for(size_t i=0; i<commands.size(); ++i){
        set_command(i, commands[i]);
    }
  // ...
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state){
    release_interfaces();
    // The controller should be properly shutdown during this
    // ...
    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state){
  // Callback function for cleanup transition
  // ...
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state){
  // Callback function for shutdown transition
  // ...
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state){
  // Callback function for erroneous transition
  // ...
  return CallbackReturn::SUCCESS;
}