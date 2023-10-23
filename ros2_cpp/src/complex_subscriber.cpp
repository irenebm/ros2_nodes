#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ros2_cpp/complex_subscriber.hpp"


ComplexSubscriber::ComplexSubscriber()
: Node("minimal_subscriber")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
  "topic", 10, std::bind(&ComplexSubscriber::topic_callback, this, _1));
}

void ComplexSubscriber::topic_callback(const std_msgs::msg::String & msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComplexSubscriber>());
  rclcpp::shutdown();
  return 0;
}
