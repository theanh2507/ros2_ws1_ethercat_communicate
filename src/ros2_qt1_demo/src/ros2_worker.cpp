#include "ros2_worker.h"

ROS2Worker::ROS2Worker(QObject *parent) : QObject(parent), running(true) 
{
    node = rclcpp::Node::make_shared("qt_ethercat_jog_node");

    velocity_publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controller/commands", 10);
    position_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/trajectory_controller/joint_trajectory", 10);
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

void ROS2Worker::publishVelocity(double v1, double v2, double v3, double v4, double v5, double v6) {
    if (!rclcpp::ok()) return;

    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {v1, v2, v3, v4, v5, v6};
    
    velocity_publisher->publish(message);
    RCLCPP_INFO(node->get_logger(), "Published: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", v1, v2, v3, v4, v5, v6);
}

void ROS2Worker::publishPosition(double p1, double p2, double p3, double p4, double p5, double p6) {
    if (!rclcpp::ok()) return;

    auto message = trajectory_msgs::msg::JointTrajectory();

    message.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(p1);                          // push back: them phan tu vao cuoi vector
    point.positions.push_back(p2);
    point.positions.push_back(p3);
    point.positions.push_back(p4);
    point.positions.push_back(p5);
    point.positions.push_back(p6);

    point.time_from_start.sec = 5; 
    point.time_from_start.nanosec = 0;

    message.points.push_back(point);

    position_publisher->publish(message);
    
    RCLCPP_INFO(node->get_logger(), "Published Position: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] to all joints in 4s", p1, p2, p3, p4, p5, p6);
}

void ROS2Worker::stop() {
    running = false;
}