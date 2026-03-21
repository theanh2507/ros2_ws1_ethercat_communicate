#include "displayStatuMotor.h"

DisplayStatusMotor::DisplayStatusMotor(QObject *parent) : QObject(parent) {
    node = std::make_shared<rclcpp::Node>("display_status_motor_node");

    subscription_joint_state = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&DisplayStatusMotor::jointStateCallback, this, std::placeholders::_1));

        subscription_dynamic_joint_state = node->create_subscription<control_msgs::msg::DynamicJointState>(
        "/dynamic_joint_states", 10, std::bind(&DisplayStatusMotor::dynamicJointStateCallback, this, std::placeholders::_1));
}

void DisplayStatusMotor::start() {
    rclcpp::spin(node); 
}

void DisplayStatusMotor::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    for(size_t i = 0; i < msg->name.size(); ++i) {
        QString jointName = QString::fromStdString(msg->name[i]);
        double position = msg->position[i];
        double velocity = msg->velocity[i];

        int index = jointName.right(1).toInt() - 1;
        emit jointDataReceive(index, position, velocity);
    }
}

void DisplayStatusMotor::dynamicJointStateCallback(const control_msgs::msg::DynamicJointState::SharedPtr msg)
{
    for (size_t i = 0; i < msg->joint_names.size(); ++i) 
    {
        QString joint_name = QString::fromStdString(msg->joint_names[i]);
        
        const auto& interfaces = msg->interface_values[i].interface_names;
        const auto& values = msg->interface_values[i].values;

        int mode = 0;
        int status = 0;


        for (size_t j = 0; j < interfaces.size(); ++j) 
        {
            if (interfaces[j] == "status_word")   status = static_cast<int>(values[j]);
            else if (interfaces[j] == "mode_of_operation") mode = static_cast<int>(values[j]);
        }

        int index = joint_name.right(1).toInt() - 1;

        if (index >= 0 && index < 6) {
            // qDebug() << "Motor:" << index << "Mode:" << mode << "Status:" << status;
            emit jointDataReceiveDynamic(index, status, mode);
        }
    }
}