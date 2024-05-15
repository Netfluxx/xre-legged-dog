#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class PositionPublisher : public rclcpp::Node
{
public:
    PositionPublisher()
    : Node("position_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("position_controller/commands", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&PositionPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Float64();
        message.data = 1.5; // Command to move to 1.5 radians, adjust as needed
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPublisher>());
    rclcpp::shutdown();
    return 0;
}
