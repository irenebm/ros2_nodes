#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;


class ComplexSubscriber : public rclcpp::Node
{
  public:
    ComplexSubscriber();
  private:
    void topic_callback(const std_msgs::msg::String & msg) const;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
