#ifndef ROS2_WORKER_H
#define ROS2_WORKER_H

#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class ROS2Worker : public QObject {
    Q_OBJECT
public:
    explicit ROS2Worker(QObject *parent = nullptr);
    virtual ~ROS2Worker();                     

    void publishVelocity(double v1, double v2, double v3, double v4, double v5, double v6);
    void publishPosition(double p1, double p2, double p3, double p4, double p5, double p6);

public slots:
    void spin();    
    void stop();

private:
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher;                  // publish velocity for jogging
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr position_publisher;             // publish position for move to position
    bool running;
};

#endif