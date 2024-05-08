#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rate.hpp>
#include <memory>
#include <chienpanze_controller_manager/controller_manager.hpp>
#include <chienpanze_hw_interface/chienpanze_hw_interface.hpp>

int main(int argc, char ** argv){
    rclcpp::init(arc, argv);
    auto node = std::make_shared<rclcpp::Node>("chienpanze_controller_manager");
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    auto chienpanze_hardware_interface = std::make_shared<chienpanze_hw_interface::chienpanze_hw_interface>(node);

    controller_manager::ControllerManager cm(chienpanze_hardware_interface, node);

    rclcpp::Rate rate(100); //should be 100Hz

    executor->add_node(chienpanze_hw_interface->get_node_base_interface());
    executor->add_node(controller_manager);

    while(rclcpp::ok()){
        executor->spin();
        controller_manager->update();
    }
    rclcpp::shutdown();
    return 0;
}