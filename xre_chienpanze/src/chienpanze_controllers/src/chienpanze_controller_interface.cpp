#include <controller_interface/controller_interface.hpp>
#include <rclcpp.lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rclcpp/qos.hpp"

#include "chienpanze_controller_interface/chienpanze_controller_interface.hpp"

#include <algorithm>
#include <vector>
#include <string>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;[]()

controller_interface::CallbackReturn on_init(){
    // declare and get parameters needed for controller initialization
    // allocate memory that will exist for the life of the controller

    // set positions at zero
    // the parameter values for the joints, command_interfaces and state_interfaces should be declared and accessed
     return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state){
    // declare and get parameters needed for controller operations
    // setup realtime buffers, ROS publishers, and ROS subscribers/publishers
    // called when the controller is set to the inactive state 
    // should read reconfigurable parameters 
    
    //SHOULD I RECEIVE DATA AS INPUTS HERE ?
    

    //command_interface_types_.clear();
    for(const std::string& joint_name : joint_names_){
        std::string interface_name = joint_name + "/position";

    }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration command_interface_configuration(){
  //called after on_configure()
  //returns a list opf InterfaceCOnfiguration objets to indicate which 
  //command interfaces the controller needs to operate
  //each command interface is uniquely indentified by its name and its interface type.
  //if a requested interface is not offered by a loaded hardware interface, then the controller wil fail.
    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    // add required command interface to `conf` by specifying their names and interface types.
    // ..conf.names.reserve(joint_names_.size() * command_interface_types_.size());
    for (const auto& joint_name : joint_names_){
        for (const auto& interface_type : command_interface_types_){
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }

  return conf;
}

controller_interface::InterfaceConfiguration state_interface_configuration() {
    //called after the command interface configuration
    //returns a list of InterfaceConfiguration objects representing th required state interfaces to operate
    controller_interface::InterfaceConfiguration conf;
    // add required state interface to `conf` by specifying their names and interface types.
    // ..

    controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
    conf.names.reserve(joint_names_.size() * state_interface_types_.size());
    for(const auto& joint_name : joint_names_){
        for(const auto& interface_type : state_interface_types_){
            conf.names.push_back(joint_name + "/" + interface_type);
        }
    }
    return conf
}

controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state){
//perform specific safety checks
  // Handle controller restarts and dynamic parameter updating
  // ...
    //set command interfaces to 0

    // clear out vectors in case of restart
//   joint_position_command_interface_.clear();
//   joint_velocity_command_interface_.clear();
//   joint_position_state_interface_.clear();
//   joint_velocity_state_interface_.clear();

  return CallbackReturn::SUCCESS;
}

std::vector<double> ChienpanzeController::get_current_positions(){
    std::vector<double> positions = {};
    return positions;
}

std::vector<double> ChienpanzeController::compute_position_commands(const std::vector<double>& current_positions){
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
    //part of the realtime main control loop
  // Read controller inputs values from state interfaces
  // Calculate controller output values and write them to command interfaces
  //Normally, the reference is accessed via a ROS 2 subscriber.
  //Since the subscriber runs on the non-realtime thread, a realtime buffer is used to 
  //a transfer the message to the realtime thread. 
    std::vector<double> current_positions = get_current_positions();
    std::vector<double> commands = compute_position_commands(current_positions);

    // for(size_t i=0; i<commands.size(); ++i){
    //     set_command(i, commands[i]);
    // }

    if(new_msg){
        start_time_ = time;
        new_msg = false;
    }

    //calculate error from the readings of the sate interfaces
    //set new goal for each motor


  // ...
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state){
    // The controller should be properly shutdown during this state transition
    //it is importatnt to release the claimed command interface in this method, so other controllers can use
    //them if needed --> done with the release_interfaces function
    // ...
    release_interfaces();

    return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state){
  // Callback function for cleanup transition
  //free up allocated memory
  // ...
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state){
  // Callback function for shutdown transition
  // ...
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state){
  // called when the managed node fails a state transition. Should never happen.
  // ...
  return CallbackReturn::SUCCESS;
}


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    chienpanze_controller_interface::ChienpanzeControllerInterface, controller_interface::SystemInterface)
