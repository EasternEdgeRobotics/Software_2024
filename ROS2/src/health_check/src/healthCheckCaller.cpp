#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HealthCheckCaller : public rclcpp::Node
{
    public:
        HealthCheckCaller() : Node("healthCheckCaller"), count_(0) {
            publisher_ = this->create_publisher<std_msgs::msg::String>("healthCheck", 10);
            timer_ = this->create_wall_timer(10000ms, std::bind(&HealthCheckCaller::timer_callback, this));
        }

    private:
        void timer_callback() {
                auto message = std_msgs::msg::String();
                message.data = "check";
                publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HealthCheckCaller>());
    rclcpp::shutdown();
    return 0;
}