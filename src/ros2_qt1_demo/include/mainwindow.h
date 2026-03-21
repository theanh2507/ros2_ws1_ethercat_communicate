#ifndef MAINWINDOW_H
#define MAINWINDOW_H




#include <QMainWindow>
#include <QDebug>
#include <QProcess>
#include <QThread>

#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "ros2_worker.h"
#include "displayStatuMotor.h"

#include <std_msgs/msg/string.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);  
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    ROS2Worker *worker;
    QThread *rosThread;

    DisplayStatusMotor *displayStatusMotor;
    QThread *displayStatusMotorThread;


    QProcess *startServiceEtherCATProcess = nullptr;
    

    QProcess *modeOfOperationProcess = nullptr;


    std::shared_ptr<rclcpp::Node> nodePubVelHome;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publishVelHomeToRos;                 // publish vel home from GUI to ros2

    std::shared_ptr<rclcpp::Node> nodeMode;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisherMode;


    void onButtonClicked();
    void startServiceEtherCAT();
    
    void enableMotor();
    void disableMotor();
    void resetFaultError();

    // void modeOfOperation(uint8_t mode);
    // void executeModeSwitch(uint8_t mode, QString wsPath);

    void modeOfOperation(uint8_t mode, uint8_t slave);
    void executeModeSwitch(uint8_t mode, uint8_t slave, QString wsPath);    

    void jogNDirection();
    void jogPDirection();
    void stopMotor();

    void moveToPosition();

    void sendVelHomeToRos();
};
#endif // MAINWINDOW_H
