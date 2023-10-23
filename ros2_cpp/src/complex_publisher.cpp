#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ros2_cpp/complex_publisher.hpp"


ComplexPublisher::ComplexPublisher()
: Node("minimal_publisher"), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(
  500ms, std::bind(&ComplexPublisher::timer_callback, this));
}

void ComplexPublisher::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComplexPublisher>());
  rclcpp::shutdown();
  return 0;
}
