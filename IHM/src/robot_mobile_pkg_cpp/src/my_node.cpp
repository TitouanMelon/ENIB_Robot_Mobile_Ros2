#include "../include/mainwindow.hpp"
#include <cstdio>
#include <QApplication>

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;


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
