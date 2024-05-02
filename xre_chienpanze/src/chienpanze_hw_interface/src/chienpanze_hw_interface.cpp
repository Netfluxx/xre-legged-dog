#include "chienpanze_hw_interface/chienpanze_hw_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <sched.h>
#include <sys/mman.h>

#include <time.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pi3hat.h"

#define DEG_TO_RAD 0.01745329251994329576923690768489

namespace chienpanze_hw_interface{
    hardware_interface::CallbackReturn ChienpanzeHardwareInterface::on_init(const hardware_interface::HardwareInfo &info){
        // Check if the required resources are available
        if(info_.hardware_parameters.empty()){
            RCLCPP_ERROR(rclcpp::get_logger("ChienpanzeHardwareInterface"), "No hardware parameters specified");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (hardware_interface::SystemInterface::on_init(info) !=hardware_interface::CallbackReturn::SUCCESS){
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_state_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        hw_command_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_kps_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_command_kds_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for(const hardware_interface::ComponentInfo &joint : info_.joints){
            if("moteus" == joint.parameters.at("CanProtocol::MOTEUS")){
                hw_actuator_can_protocols_.push_back(CanProtocol::MOTEUS);
            }
            else{
                RCLCPP_ERROR(rclcpp::get_logger("ChienpanzeHardwareInterface"), "an_protocol parameter does not match moteus protocol");
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Set params for each joint
            hw_actuator_can_channels_.push_back(std::stoi(joint.parameters.at("can_channel")));
            hw_actuator_can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
            hw_actuator_position_scales_.push_back(std::stod(joint.parameters.at("position_scale")));
            hw_actuator_velocity_scales_.push_back(std::stod(joint.parameters.at("velocity_scale")));
            hw_actuator_effort_scales_.push_back(std::stod(joint.parameters.at("effort_scale")));
            hw_actuator_kp_scales_.push_back(std::stod(joint.parameters.at("kp_scale")));
            hw_actuator_kd_scales_.push_back(std::stod(joint.parameters.at("kd_scale")));
            hw_actuator_axis_directions_.push_back(std::stoi(joint.parameters.at("axis_direction")));
            hw_actuator_position_offsets_.push_back(std::stod(joint.parameters.at("position_offset")));

            // Set limits for each joint
            hw_actuator_position_mins_.push_back(std::stod(joint.parameters.at("position_min")));
            hw_actuator_position_maxs_.push_back(std::stod(joint.parameters.at("position_max")));
            hw_actuator_velocity_maxs_.push_back(std::stod(joint.parameters.at("velocity_max")));
            hw_actuator_effort_maxs_.push_back(std::stod(joint.parameters.at("effort_max")));
            hw_actuator_kp_maxs_.push_back(std::stod(joint.parameters.at("kp_max")));
            hw_actuator_kd_maxs_.push_back(std::stod(joint.parameters.at("kd_max")));
        }

        // Configure the Pi3Hat CAN for FD mode with bitrate switching and automatic retranmission
        mjbots::pi3hat::Pi3Hat::CanConfiguration can_config;
        can_config.fdcan_frame = true;
        can_config.bitrate_switch = true;
        can_config.automatic_retransmission = true;

        // Initialize the Pi3Hat input without IMU
        pi3hat_input_ = mjbots::pi3hat::Pi3Hat::Input();
        mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> rx_can_frames_span_(rx_can_frames_, MAX_NUM_CAN_FRAMES); 
        pi3hat_input_.rx_can = rx_can_frames_span_;
        mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame> tx_can_frames_span_(tx_can_frames_, info_.joints.size()); 
        pi3hat_input_.tx_can = tx_can_frames_span_;

        // Set up the CAN configuration
        for (uint i = 0; i < info_.joints.size(); i++){
            config.can[hw_actuator_can_channels_[i] - 1] = can_config;
            pi3hat_input_.tx_can[i].id = hw_actuator_can_ids_[i];
            pi3hat_input_.tx_can[i].bus = hw_actuator_can_channels_[i];
            pi3hat_input_.tx_can[i].expect_reply = true;
            pi3hat_input_.tx_can[i].size = 64;
        }

        // Initialize the Pi3Hat
        pi3hat_ = new mjbots::pi3hat::Pi3Hat(config);

        // MAGIE NOIRE !!!
        // Configure realtime scheduling
        {
            int realtime_cpu = 0;
            cpu_set_t cpuset = {};
            CPU_ZERO(&cpuset);
            CPU_SET(realtime_cpu, &cpuset);

            const int r = ::sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
            if (r < 0){
                throw std::runtime_error("Error setting CPU affinity");
            }

            std::cout << "Affinity set to " << realtime_cpu << "\n";
        }
        {
            struct sched_param params = {};
            params.sched_priority = 10;
            const int r = ::sched_setscheduler(0, SCHED_RR, &params);
            if (r < 0){
                throw std::runtime_error("Error setting realtime scheduler");
            }
        }
        {
            const int r = ::mlockall(MCL_CURRENT | MCL_FUTURE);
            if (r < 0){
                throw std::runtime_error("Error locking memory");
            }
        }
    }

    std::vector<hardware_interface::StateInterface> ChienpanzeHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Add joint state interfaces
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_efforts_[i]));
        }

        return state_interfaces;
    }

    hardware_interface::CallbackReturn ChienpanzeHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // reset values always when configuring hardware
        for (uint i = 0; i < hw_state_positions_.size(); i++)
        {
            hw_state_positions_[i] = 0;
            hw_state_velocities_[i] = 0;
            hw_state_efforts_[i] = 0;
            hw_command_positions_[i] = 0;
            hw_command_velocities_[i] = 0;
            hw_command_efforts_[i] = 0;
            hw_command_kps_[i] = 0;
            hw_command_kds_[i] = 0;
        }

        RCLCPP_INFO(rclcpp::get_logger("ChienpanzeHardwareInterface"), "Successfully configured!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::CommandInterface> ChienpanzeHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "kp", &hw_command_kps_[i]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, "kd", &hw_command_kds_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn ChienpanzeHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/){
        RCLCPP_INFO(rclcpp::get_logger("ChienpanzeHardwareInterface"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ChienpanzeHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/){
        
    // determine message to send
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ChienpanzeHardwareInterface::read(
        const rclcpp::Time & /*time*/, the rclcpp::Duration & /*period*/)
    {
        // Reading data involves receiving and processing the CAN frames
        std::vector<mjbots::pi3hat::CanFrame> rx_frames(MAX_NUM_CAN_FRAMES);
        mjbots::pi3hat::Pi3Hat::Input input;
        input.rx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(rx_frames.data(), rx_frames.size());

        auto output = pi3hat_.Cycle(input);
        if (output.rx_can_size > 0) {
            for (size_t i = 0; i < output.rx_can_size; ++i) {
                const auto& frame = rx_frames[i];
                // Process the received frame, e.g., update internal state based on CAN ID
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ChienpanzeHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        mjbots::pi3hat::Pi3Hat::Input input;
        std::vector<mjbots::pi3hat::CanFrame> tx_frames(info_.joints.size());

        for (size_t i = 0; i < info_.joints.size(); ++i) {
            if (std::isnan(hw_command_positions_[i])) {
                RCLCPP_WARN(rclcpp::get_logger("ChienpanzeHardwareInterface"), "NaN command for actuator %s", info_.joints[i].name.c_str());
                continue;
            }

            // Convert command to Moteus format and prepare CAN frame
            tx_frames[i].id = hw_actuator_can_ids_[i];
            tx_frames[i].bus = hw_actuator_can_channels_[i];

            // Setting the command type and value
            tx_frames[i].data[0] = 0x020;  // Position command type --> a verifier
            int32_t position_command = static_cast<int32_t>(hw_command_positions_[i] * POSITION_SCALE);  // POSITION_SCALE converts to desired units
            std::memcpy(&tx_frames[i].data[1], &position_command, sizeof(position_command));
            tx_frames[i].size = 1 + sizeof(position_command);  // Command type byte + 4 bytes of position

            tx_frames[i].expect_reply = true;  // If you expect a reply from the motor
        }

        input.tx_can = mjbots::pi3hat::Span<mjbots::pi3hat::CanFrame>(tx_frames.data(), tx_frames.size());

        // Execute the cycle with your configured input
        auto output = pi3hat_.Cycle(input);
        
        // Process output, handle errors, and/or log results

        return hardware_interface::return_type::OK;
    }

    

    
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    chienpanze_hw_interface::ChienpanzeHardwareInterface, hardware_interface::SystemInterface)