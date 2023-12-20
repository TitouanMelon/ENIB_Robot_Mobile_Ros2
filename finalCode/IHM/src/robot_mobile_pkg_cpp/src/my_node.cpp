#include "../include/mainwindow.hpp"
#include <cstdio>
#include <QApplication>

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
/*
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

//*/


int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    printf("Initializing ros2 communication functionalities...\n");
    rclcpp::init(argc, argv);
   //auto node = std::make_shared<MinimalPublisher>();
   //rclcpp::spin(node);
    printf("ros2 communication functionalities started\n");


    QApplication a(argc, argv);

    printf("Launching GUI ...\n");
    MainWindow w;

    w.show();
    printf("GUI Started ...\n");

    return a.exec();

 // return 0;

}
