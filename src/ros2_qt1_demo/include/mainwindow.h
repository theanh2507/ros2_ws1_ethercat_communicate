#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include "rclcpp/rclcpp.hpp"
#include <memory>

#include <QMainWindow>
#include <QDebug>
#include <QProcess>


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

    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubVel;


    QProcess *startServiceEtherCATProcess = nullptr;
    
    QProcess *enableMotorProcess = nullptr;
    QProcess *disableMotorProcess = nullptr;
    QProcess *resetFaultErrorProcess = nullptr;

    QProcess *modeOfOperationProcess = nullptr;


    void onButtonClicked();

    void startServiceEtherCAT();
    void enableMotor();
    void disableMotor();
    void resetFaultError();

    void modeOfOperation(uint8_t mode);

    void jogNDirection();
    void jogPDirection();
    void stopMotor();
};
#endif // MAINWINDOW_H
