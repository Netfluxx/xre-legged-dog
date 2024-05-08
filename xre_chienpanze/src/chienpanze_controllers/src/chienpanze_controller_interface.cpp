#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/types/lifecycle_state_names.hpp"


#include "chienpanze_controller_interface/chienpanze_controller_interface.hpp"
//#include "chienpanze_controller_interface/ChienpanzeController.hpp"
#include "chienpanze_controller_interface/chienpanze_controller_interface.hpp"


#include "controller_interface/controller_interface_base.hpp"

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <algorithm>
#include <vector>
#include <string>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace chienpanze_controller_interface{

	controller_interface::CallbackReturn ChienpanzeController::on_init(){
		// declare and get parameters needed for controller initialization
		// allocate memory that will exist for the life of the controller
		// set positions at zero
		// the parameter values for the joints, command_interfaces and state_interfaces should be declared and accessed
		joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
		command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
		state_interface_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
		//TO DO : SET POSITIONS AND VELOCITIES AT 0
		return CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state){
		// declare and get parameters needed for controller operations
		// setup realtime buffers, ROS publishers, and ROS subscribers/publishers
		// called when the controller is set to the inactive state 
		// should read reconfigurable parameters 
		
		//SETUP ROS SUBSCRI;BER TO INPUT TOPIC


	return CallbackReturn::SUCCESS;
	}

	controller_interface::InterfaceConfiguration ChienpanzeController::command_interface_configuration() const{
	//called after on_configure()
	//returns a list opf InterfaceCOnfiguration objets to indicate which 
	//command interfaces the controller needs to operate
	//each command interface is uniquely indentified by its name and its interface type.
	//if a requested interface is not offered by a loaded hardware interface, then the controller wil fail.
		// add required command interface to `conf` by specifying their names and interface types.
		controller_interface::InterfaceConfiguration conf;
		conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

		conf.names.reserve(joint_names_.size() * command_interface_types_.size());
		// Assuming you need position and velocity commands
		for(const auto& joint_name : joint_names_){
			for(const auto& interface_type : command_interface_types_){
				conf.names.push_back(joint_name + "/" + interface_type);
			}
		}
		// for (const auto& joint_name : joint_names_) {
		//     config.names.push_back(joint_name + "/position");
		//     config.names.push_back(joint_name + "/velocity");
		//     config.names.push_back(joint_name + "/effort");  // Is it needed?
		// }
		return conf;
	}

	controller_interface::InterfaceConfiguration ChienpanzeController::state_interface_configuration() const{
		//called after the command interface configuration
		//returns a list of InterfaceConfiguration objects representing th required state interfaces to operate
		// add required state interface to `conf` by specifying their names and interface types.
		// ..

		controller_interface::InterfaceConfiguration conf;
		conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
		conf.names.reserve(joint_names_.size() * state_interface_types_.size());
		for(const auto& joint_name : joint_names_){
			for(const auto& interface_type : state_interface_types_){
				conf.names.push_back(joint_name + "/" + interface_type);
			}
		}
		return conf;
	}

	controller_interface::CallbackReturn ChienpanzeController::on_activate(const rclcpp_lifecycle::State &previous_state){
	//perform specific safety checks
	// Handle controller restarts and dynamic parameter updating
	// ...

		// clear out vectors in case of restart
		joint_position_command_interface_.clear();
		joint_velocity_command_interface_.clear();
		joint_position_state_interface_.clear();
		joint_velocity_state_interface_.clear();
		// assign command interfaces
		for (auto & interface : command_interfaces_){
			command_interface_map_[interface.get_interface_name()]->push_back(interface);
		}

		// assign state interfaces
		for (auto & interface : state_interfaces_){
			state_interface_map_[interface.get_interface_name()]->push_back(interface);
		}
		return CallbackReturn::SUCCESS;
	}



	controller_interface::return_type ChienpanzeController::update(const rclcpp::Time &time, const rclcpp::Duration &period){
		//part of the realtime main control loop
		// Read controller inputs values from state interfaces
		// Calculate controller output values and write them to command interfaces
		//Normally, the reference is accessed via a ROS 2 subscriber.
		//Since the subscriber runs on the non-realtime thread, a realtime buffer is used to 
		//a transfer the message to the realtime thread. 

		// ...
		return controller_interface::return_type::OK;
	}

	controller_interface::CallbackReturn ChienpanzeController::on_deactivate(const rclcpp_lifecycle::State &previous_state){
		// The controller should be properly shutdown during this state transition
		//it is importatnt to release the claimed command interface in this method, so other controllers can use
		//them if needed --> done with the release_interfaces function
		// ...
		release_interfaces();
		//command interfaces
		joint_position_command_interface_.clear();
		joint_velocity_command_interface_.clear();
		//state interfaces
		joint_position_state_interface_.clear();
		joint_velocity_state_interface_.clear();
		return CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn ChienpanzeController::on_cleanup(const rclcpp_lifecycle::State &previous_state){
	// Callback function for cleanup transition
	//free up allocated memory
	// ...
	return CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn ChienpanzeController::on_shutdown(const rclcpp_lifecycle::State &previous_state){
	// Callback function for shutdown transition
	// ...
	return CallbackReturn::SUCCESS;
	}

	controller_interface::CallbackReturn ChienpanzeController::on_error(const rclcpp_lifecycle::State &previous_state){
	// called when the managed node fails a state transition. Should never happen.
	// ...
	return CallbackReturn::SUCCESS;
	}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(chienpanze_controller_interface::ChienpanzeController, controller_interface::ControllerInterface)
