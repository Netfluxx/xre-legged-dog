#ifndef CHIENPANZE_HW_INTERFACE__CHIENPANZE_HW_INTERFACE_HPP_
#define CHIENPANZE_HW_INTERFACE__CHIENPANZE_HW_INTERFACE_HPP_


#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


#include "pi3hat/pi3hat.h"


#define MAX_NUM_CAN_FRAMES 12 // ??? also for can-fd ??

namespace chienpanze_hw_interface{
    class ChienpanzeHardwareInterface : public hardware_interface::SystemInterface{
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(ChienpanzeHardwareInterface)

            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo &info) override;

            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State &previous_state) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State &previous_state) override;

            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State &previous_state) override;

            hardware_interface::return_type read(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;

            hardware_interface::return_type write(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;
        private:
            mjbots::pi3hat::Pi3Hat *pi3hat_;
            mjbots::pi3hat::Pi3Hat::Input pi3hat_input_;
            mjbots::pi3hat::Attitude attitude_;
            mjbots::pi3hat::CanFrame tx_can_frames_[MAX_NUM_CAN_FRAMES];
            mjbots::pi3hat::CanFrame rx_can_frames_[MAX_NUM_CAN_FRAMES];

            // Actuator CAN configuration
            std::vector<int> hw_actuator_can_channels_;
            std::vector<int> hw_actuator_can_ids_;
            std::vector<CanProtocol> hw_actuator_can_protocols_;

            // Actuator parameters
            std::vector<double> hw_actuator_position_scales_;
            std::vector<double> hw_actuator_velocity_scales_;
            std::vector<double> hw_actuator_effort_scales_;
            std::vector<double> hw_actuator_kp_scales_;
            std::vector<double> hw_actuator_kd_scales_;
            std::vector<int> hw_actuator_axis_directions_;
            std::vector<double> hw_actuator_position_offsets_;

            // Actuator limits
            std::vector<double> hw_actuator_position_mins_; 
            std::vector<double> hw_actuator_position_maxs_;
            std::vector<double> hw_actuator_velocity_maxs_;
            std::vector<double> hw_actuator_effort_maxs_;
            std::vector<double> hw_actuator_kp_maxs_;
            std::vector<double> hw_actuator_kd_maxs_;

            // Actuator states
            std::vector<double> hw_state_positions_;
            std::vector<double> hw_state_velocities_;
            std::vector<double> hw_state_efforts_;

            // Actuator commands
            std::vector<double> hw_command_positions_;
            std::vector<double> hw_command_velocities_;
            std::vector<double> hw_command_efforts_;
            std::vector<double> hw_command_kps_;
            std::vector<double> hw_command_kds_;
    
    };
}

#endif