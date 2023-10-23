#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;


class SimpleImagePublisher : public rclcpp::Node
{
  public:
    SimpleImagePublisher()
    : Node("simple_image_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&SimpleImagePublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      cv::Mat image = cv::imread(ament_index_cpp::get_package_share_directory("ros2_images") + "/resources/ros2_logo.png", cv::IMREAD_COLOR);
      auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

      RCLCPP_INFO(this->get_logger(), "I'm sending the image :) [size: ('%d','%d')]", message->height, message->width);

      publisher_->publish(*message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
