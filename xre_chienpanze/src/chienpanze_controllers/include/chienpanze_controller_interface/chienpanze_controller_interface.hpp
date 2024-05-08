
#ifndef CHIENPANZE_CONTROLLER_INTERFACE_HPP_
#define CHIENPANZE_CONTROLLER_INTERFACE_HPP_

#include <controller_interface/controller_interface.hpp>
#include "controller_interface/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"

#include <vector>
#include <unordered_map>
#include <string>

class ChienpanzeController : public controller_interface::ControllerInterface {
    public:
    	ChienpanzeController() = default;
        ~ChienpanzeController() override = default;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    
    private:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

		//command interfaces
		std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
			joint_position_command_interface_;
		std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
			joint_velocity_command_interface_;
			//state interfaces
		std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
			joint_position_state_interface_;
		std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
			joint_velocity_state_interface_;

        rclcpp::Time start_time_;
        bool new_msg = false;

		std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
			state_interface_map_ = {
				{"position", &joint_position_state_interface_},
				{"velocity", &joint_velocity_state_interface_}};

		std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
    		command_interface_map_ = {
				{"position", &joint_position_command_interface_},
				{"velocity", &joint_velocity_command_interface_}};
};


//joint_position_command_interface_
//joint_velocity_command_interface_
//joint_position_sate_interface
//joint_velocity_state_interface


#endif // CHIENPANZE_CONTROLLER_INTERFACE_HPP_