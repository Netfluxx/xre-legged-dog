
#ifndef CHIENPANZE_CONTROLLER_INTERFACE__CHIENPANZE_CONTROLLER_INTERFACE_HPP_
#define CHIENPANZE_CONTROLLER_INTERFACE__CHIENPANZE_CONTROLLER_INTERFACE_HPP_

#include <controller_interface/controller_interface.hpp>
#include <rclcpp.lifecycle/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"

#include <vector>
#include <string>

class SimpleController : public controller_interface::ControllerInterface {
    public:
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
        std::vector<double> get_current_positions();
        std::vector<double> compute_position_commands(const std::vector<double>& current_positions);
        void set_command(size_t& joint_index, double& command_value);

        std::vector<double> command_positions_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        rclcpp::Time start_time_;
}



#endif