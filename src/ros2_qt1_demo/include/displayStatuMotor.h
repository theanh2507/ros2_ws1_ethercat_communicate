#ifndef DISPLAY_STATUS_MOTOR_H
#define DISPLAY_STATUS_MOTOR_H

#include <QObject>
#include <QDebug>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/msg/dynamic_joint_state.hpp"

class DisplayStatusMotor : public QObject{
    Q_OBJECT
    public:
        explicit DisplayStatusMotor(QObject *parent = nullptr);
        void start();
    signals:
        void jointDataReceive(int index, double position, double velocity);
        void jointDataReceiveDynamic(int index, int status, int mode);

    private:
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_state;
        rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr subscription_dynamic_joint_state;

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void dynamicJointStateCallback(const control_msgs::msg::DynamicJointState::SharedPtr msg);
};

#endif