#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class HealthCheckResponse : public rclcpp::Node
{
    public:
        HealthCheckResponse() : Node("healthCheckResponse") {
            this->declare_parameter("host", rclcpp::PARAMETER_STRING);
            subscription_ = this->create_subscription<std_msgs::msg::String>("healthCheck", 10, std::bind(&HealthCheckResponse::topic_callback, this, _1));
            publisher_ = this->create_publisher<std_msgs::msg::String>("healthCheck", 10);
        }

    private:
        void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
            if (strcmp(msg->data.c_str(), "check") == 0) {
                auto message = std_msgs::msg::String();
                message.data=this->get_parameter("host").as_string();
                publisher_->publish(message);
            }
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HealthCheckResponse>());
    rclcpp::shutdown();
    return 0;
}