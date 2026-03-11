#include "ros2_worker.h"

ROS2Worker::ROS2Worker(QObject *parent) : QObject(parent), running(true) {
    node = rclcpp::Node::make_shared("qt_ethercat_jog_node");

    publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controller/commands", 10);
}

ROS2Worker::~ROS2Worker() {
    stop();
}

void ROS2Worker::spin() {
    // kiem tra cac callback trong thread 
    rclcpp::WallRate loop_rate(100); // Chạy vòng lặp 100Hz
    while (rclcpp::ok() && running) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}

void ROS2Worker::publishVelocity(double v1) {
    if (!rclcpp::ok()) return;

    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {v1};
    
    publisher->publish(message);
    RCLCPP_INFO(node->get_logger(), "Published: [%.2f]", v1);
}

void ROS2Worker::stop() {
    running = false;
}