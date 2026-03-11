#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    startServiceEtherCAT();
    // modeOfOperation(9); // Setting mode of operation to Cyclic Synchronous Position (CSV)

    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    connect(ui->pushButton_2, &QPushButton::clicked, this, &MainWindow::enableMotor);
    connect(ui->pushButton_5, &QPushButton::clicked, this, &MainWindow::disableMotor);
    connect(ui->pushButton_6, &QPushButton::clicked, this, &MainWindow::resetFaultError);

    connect(ui->pushButton_3, &QPushButton::pressed, this, &MainWindow::jogPDirection);
    connect(ui->pushButton_3, &QPushButton::released, this, &MainWindow::stopMotor);

    connect(ui->pushButton_4, &QPushButton::pressed, this, &MainWindow::jogNDirection);
    connect(ui->pushButton_4, &QPushButton::released, this, &MainWindow::stopMotor);


    // change mod eof operation
    connect(ui->pushButton_8, &QPushButton::clicked, this, [this](){ modeOfOperation(9); }); // Cyclic Synchronous Position (CSV)
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onButtonClicked()
{
    qDebug() << "Button was clicked!";
}

void MainWindow::startServiceEtherCAT()
{
    qDebug() << "Starting EtherCAT service...";
    
    if (!startServiceEtherCATProcess) {
        startServiceEtherCATProcess = new QProcess(this);
    }

    QString wsPath = "/home/theanh/ros2_ws1"; 
    
    QString program = "/bin/bash";
    QStringList arguments;
    
    QString command = QString("source /opt/ros/humble/setup.bash && "
                              "source %1/install/setup.bash && "
                              "ros2 run ethercat_manager ethercat_sdo_srv_server").arg(wsPath);
    
    arguments << "-c" << command;

    startServiceEtherCATProcess->start(program, arguments);
}

void MainWindow::enableMotor()
{
    if(!enableMotorProcess) {
        enableMotorProcess = new QProcess(this);
    }
    QString wsPath = "/home/theanh/ros2_ws1";
    QString program = "/bin/bash";
    
    QString command = QString(
        "source /opt/ros/humble/setup.bash && "
        "source %1/install/setup.bash && "
        "ros2 service call /ethercat_manager/set_sdo ethercat_msgs/srv/SetSdo "
        "\"{master_id: 0, slave_position: 0, sdo_index: 0x6040, sdo_subindex: 0, sdo_data_type: 'uint16', sdo_value: '15'}\""
    ).arg(wsPath);

    QStringList arguments;
    arguments << "-c" << command;

    connect(enableMotorProcess, &QProcess::readyReadStandardOutput, this, [this](){
        qDebug() << "Enable Motor Output:" << enableMotorProcess->readAllStandardOutput().trimmed();
    });

    enableMotorProcess->start(program, arguments);
    
    qDebug() << "Sending Enable command...";
}


void MainWindow::disableMotor()
{
    if(!disableMotorProcess) {
        disableMotorProcess = new QProcess(this);
    }
    QString wsPath = "/home/theanh/ros2_ws1";
    QString program = "/bin/bash";
    
    QString command = QString(
        "source /opt/ros/humble/setup.bash && "
        "source %1/install/setup.bash && "
        "ros2 service call /ethercat_manager/set_sdo ethercat_msgs/srv/SetSdo "
        "\"{master_id: 0, slave_position: 0, sdo_index: 0x6040, sdo_subindex: 0, sdo_data_type: 'uint16', sdo_value: '6'}\""
    ).arg(wsPath);

    QStringList arguments;
    arguments << "-c" << command;

    connect(disableMotorProcess, &QProcess::readyReadStandardOutput, this, [this](){
        qDebug() << "Disable Motor Output:" << disableMotorProcess->readAllStandardOutput().trimmed();
    });

    disableMotorProcess->start(program, arguments);
    
    qDebug() << "Sending Disable command...";
}


void MainWindow::resetFaultError()
{
    if(!resetFaultErrorProcess) {
        resetFaultErrorProcess = new QProcess(this);
    }
    QString wsPath = "/home/theanh/ros2_ws1";
    QString program = "/bin/bash";
    
    QString command = QString(
        "source /opt/ros/humble/setup.bash && "
        "source %1/install/setup.bash && "
        "ros2 service call /ethercat_manager/set_sdo ethercat_msgs/srv/SetSdo "
        "\"{master_id: 0, slave_position: 0, sdo_index: 0x6040, sdo_subindex: 0, sdo_data_type: 'uint16', sdo_value: '128'}\""
    ).arg(wsPath);

    QStringList arguments;
    arguments << "-c" << command;

    connect(resetFaultErrorProcess, &QProcess::readyReadStandardOutput, this, [this](){
        qDebug() << "Reset Fault Error Output:" << resetFaultErrorProcess->readAllStandardOutput().trimmed();
    });

    resetFaultErrorProcess->start(program, arguments);
    
    qDebug() << "Sending Reset Fault Error command...";
}

void MainWindow::modeOfOperation(uint8_t mode)
{
    if(!modeOfOperationProcess) {
        modeOfOperationProcess = new QProcess(this);
    }

    QString wsPath = "/home/theanh/ros2_ws1";
    QString program = "/bin/bash";

    QString command = QString(
        "source /opt/ros/humble/setup.bash && "
        "source %1/install/setup.bash && "
        "ros2 service call /ethercat_manager/set_sdo ethercat_msgs/srv/SetSdo "
        "\"{master_id: 0, slave_position: 0, sdo_index: 0x6060, sdo_subindex: 0, sdo_data_type: 'int8', sdo_value: '%2'}\""
    ).arg(wsPath).arg(mode);

    QStringList arguments;
    arguments << "-c" << command;

    modeOfOperationProcess->start(program, arguments);

    qDebug() << "Setting Mode of Operation to:" << mode;
}


void MainWindow::jogPDirection()
{
    QString velocityStr = ui->lineEdit->text();

    if(velocityStr.isEmpty()) velocityStr = "0.0";

    qDebug() << "Jogging Positive with velocity:" << velocityStr;

    QProcess *jogProcess = new QProcess(this);
    
    QString command = QString(
        // "source /opt/ros/humble/setup.bash && "
        // "source /home/theanh/ros2_ws1/install/setup.bash && "
        "ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray \"{data: [%1]}\""
    ).arg(velocityStr);

    jogProcess->start("bash", QStringList() << "-c" << command);

    // connect(jogProcess, &QProcess::finished, jogProcess, &QProcess::deleteLater);
}

void MainWindow::jogNDirection()
{
    QString velocityStr = ui->lineEdit->text();
    if(velocityStr.isEmpty()) velocityStr = "0.0";

    double v = velocityStr.toDouble();
    QString negativeVelocity = QString::number(-v);

    qDebug() << "Jogging Negative with velocity:" << negativeVelocity;

    QProcess *jogProcess = new QProcess(this);
    QString command = QString(
        // "source /opt/ros/humble/setup.bash && "
        // "source /home/theanh/ros2_ws1/install/setup.bash && "
        "ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray \"{data: [%1]}\""
    ).arg(negativeVelocity);

    jogProcess->start("bash", QStringList() << "-c" << command);
    // connect(jogProcess, &QProcess::finished, jogProcess, &QProcess::deleteLater);
}

void MainWindow::stopMotor()
{
    qDebug() << "Button released. Stopping motor...";
    
    QProcess *stopProcess = new QProcess(this);
    QString command = "ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray \"{data: [0.0]}\"";

    stopProcess->start("bash", QStringList() << "-c" << command);
    // connect(stopProcess, &QProcess::finished, stopProcess, &QProcess::deleteLater);
}
