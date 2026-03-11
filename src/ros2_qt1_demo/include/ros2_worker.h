#ifndef ROS2_WORKER_H
#define ROS2_WORKER_H

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class ROS2Worker : public QObject {
    Q_OBJECT
public:
    explicit ROS2Worker(QObject *parent = nullptr);
    virtual ~ROS2Worker();

    void publishVelocity(double v1);

public slots:
    void spin();
    void stop();

private:
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    bool running;
};

#endif