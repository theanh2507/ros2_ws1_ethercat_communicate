#ifndef MAINWINDOW_H
#define MAINWINDOW_H




#include <QMainWindow>
#include <QDebug>
#include <QProcess>
#include <QThread>

#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "ros2_worker.h"


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


    QProcess *startServiceEtherCATProcess = nullptr;
    
    QProcess *enableMotorProcess = nullptr;
    QProcess *disableMotorProcess = nullptr;
    QProcess *resetFaultErrorProcess = nullptr;

    QProcess *modeOfOperationProcess = nullptr;


    std::shared_ptr<rclcpp::Node> nodePubVelHome;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publishVelHomeToRos;                 // publish vel home from GUI to ros2


    void onButtonClicked();
    void startServiceEtherCAT();

    void enableMotor(uint8_t slave);
    void disableMotor(uint8_t slave);
    void resetFaultError(uint8_t slave);

    void modeOfOperation(uint8_t mode);

    void jogNDirection();
    void jogPDirection();
    void stopMotor();

    void moveToPosition();

    void sendVelToRos();
};
#endif // MAINWINDOW_H
