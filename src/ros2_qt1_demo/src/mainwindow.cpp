#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    startServiceEtherCAT();
    // modeOfOperation(9); // Setting mode of operation to Cyclic Synchronous Position (CSV)

    // Khởi tạo ROS 2
    rclcpp::init(0, nullptr);
    worker = new ROS2Worker();
    rosThread = new QThread(this);
    
    worker->moveToThread(rosThread);
    connect(rosThread, &QThread::started, worker, &ROS2Worker::spin);
    rosThread->start();



    // send vel homing to ros from GUI
    nodePubVelHome = rclcpp::Node::make_shared("gui_homing_publisher");
    publishVelHomeToRos = nodePubVelHome->create_publisher<std_msgs::msg::Float64MultiArray>("homing_manager_topic", 10);
    connect(ui->pushButton_11, &QPushButton::clicked, this, &MainWindow::sendVelHomeToRos);

    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
    // connect(ui->pushButton_2, &QPushButton::clicked, this, &MainWindow::enableMotor);
    // connect(ui->pushButton_5, &QPushButton::clicked, this, &MainWindow::disableMotor);
    // connect(ui->pushButton_6, &QPushButton::clicked, this, &MainWindow::resetFaultError);

    connect(ui->pushButton_2, &QPushButton::clicked, this, [this](){ 
        for(int slave = 0; slave < 6; slave++) {
            enableMotor(slave);
        }
    });

    connect(ui->pushButton_5, &QPushButton::clicked, this, [this](){ 
        for(int slave = 0; slave < 6; slave++) {
            disableMotor(slave);
        }
    });

    connect(ui->pushButton_6, &QPushButton::clicked, this, [this](){ 
        for(int slave = 0; slave < 6; slave++) {
            resetFaultError(slave);
        }
    });


    // change mode of operation
    connect(ui->pushButton_8, &QPushButton::clicked, this, [this](){ modeOfOperation(9); }); // Cyclic Synchronous Position (CSV)
    connect(ui->pushButton_7, &QPushButton::clicked, this, [this](){ modeOfOperation(8); }); // Profile Position Mode

    // Jogging buttons
    connect(ui->pushButton_3, &QPushButton::pressed, this, &MainWindow::jogPDirection);
    connect(ui->pushButton_3, &QPushButton::released, this, &MainWindow::stopMotor);

    connect(ui->pushButton_4, &QPushButton::pressed, this, &MainWindow::jogNDirection);
    connect(ui->pushButton_4, &QPushButton::released, this, &MainWindow::stopMotor);

    // Move to position button
    connect(ui->pushButton_9, &QPushButton::clicked, this, &MainWindow::moveToPosition);
}

MainWindow::~MainWindow()
{
    rosThread->quit();
    rclcpp::shutdown();
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

void MainWindow::enableMotor(uint8_t slave)
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
        "\"{master_id: 0, slave_position: %2, sdo_index: 0x6040, sdo_subindex: 0, sdo_data_type: 'uint16', sdo_value: '15'}\""
    ).arg(wsPath).arg(slave);

    QStringList arguments;
    arguments << "-c" << command;

    connect(enableMotorProcess, &QProcess::readyReadStandardOutput, this, [this](){
        qDebug() << "Enable Motor Output:" << enableMotorProcess->readAllStandardOutput().trimmed();
    });

    enableMotorProcess->start(program, arguments);
    
    qDebug() << "Sending Enable command...";
}


void MainWindow::disableMotor(uint8_t slave)
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
        "\"{master_id: 0, slave_position: %2, sdo_index: 0x6040, sdo_subindex: 0, sdo_data_type: 'uint16', sdo_value: '6'}\""
    ).arg(wsPath).arg(slave);

    QStringList arguments;
    arguments << "-c" << command;

    connect(disableMotorProcess, &QProcess::readyReadStandardOutput, this, [this](){
        qDebug() << "Disable Motor Output:" << disableMotorProcess->readAllStandardOutput().trimmed();
    });

    disableMotorProcess->start(program, arguments);
    
    qDebug() << "Sending Disable command...";
}


void MainWindow::resetFaultError(uint8_t slave)
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
        "\"{master_id: 0, slave_position: %2, sdo_index: 0x6040, sdo_subindex: 0, sdo_data_type: 'uint16', sdo_value: '128'}\""
    ).arg(wsPath).arg(slave);

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

    QString wsPath = "/home/theanh/ros2_ws1";
    QString program = "/bin/bash";
    QString activateController;
    QString deactivateController;

    if(!modeOfOperationProcess) {
        modeOfOperationProcess = new QProcess(this);
    }

    if (mode == 9) { // CSV - Velocity
        activateController = "velocity_controller";
        deactivateController = "trajectory_controller";
        qDebug() << "Switching to: Velocity Mode (CSV)";
    } else if (mode == 8) { // Profile Position
        activateController = "trajectory_controller";
        deactivateController = "velocity_controller";
        qDebug() << "Switching to: Position Mode (PP)";
    }

    QString command = QString(
        "source /opt/ros/humble/setup.bash && "
        "source %1/install/setup.bash && "
        "ros2 service call /ethercat_manager/set_sdo ethercat_msgs/srv/SetSdo "
        "\"{master_id: 0, slave_position: 0, sdo_index: 0x6060, sdo_subindex: 0, sdo_data_type: 'int8', sdo_value: '%2'}\"&&"
        "ros2 control switch_controllers --activate %3 --deactivate %4 --switch-timeout 5.0"
    ).arg(wsPath).arg(mode).arg(activateController).arg(deactivateController);

    QStringList arguments;
    arguments << "-c" << command;

    modeOfOperationProcess->start(program, arguments);

    qDebug() << "Setting Mode of Operation to:" << mode;
}


void MainWindow::jogPDirection() {
    double v1 = ui->lineEdit->text().toDouble();
    double v2 = ui->lineEdit_4->text().toDouble();
    double v3 = ui->lineEdit_5->text().toDouble();
    double v4 = ui->lineEdit_6->text().toDouble();
    double v5 = ui->lineEdit_7->text().toDouble();
    double v6 = ui->lineEdit_8->text().toDouble();
    worker->publishVelocity(v1, v2, v3, v4, v5, v6);
}

void MainWindow::jogNDirection() {
    double v1 = ui->lineEdit->text().toDouble();
    double v2 = ui->lineEdit_4->text().toDouble();
    double v3 = ui->lineEdit_5->text().toDouble();
    double v4 = ui->lineEdit_6->text().toDouble();
    double v5 = ui->lineEdit_7->text().toDouble();
    double v6 = ui->lineEdit_8->text().toDouble();
    worker->publishVelocity(-v1, -v2, -v3, -v4, -v5, -v6);
}

void MainWindow::stopMotor() {
    worker->publishVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void MainWindow::moveToPosition() {
    double p1 = ui->lineEdit_2->text().toDouble();
    double p2 = ui->lineEdit_9->text().toDouble();
    double p3 = ui->lineEdit_10->text().toDouble();
    double p4 = ui->lineEdit_11->text().toDouble();
    double p5 = ui->lineEdit_12->text().toDouble();
    double p6 = ui->lineEdit_13->text().toDouble();
    worker->publishPosition(p1, p2, p3, p4, p5, p6);
}


void MainWindow::sendVelHomeToRos() {
    double velLimit = ui->lineEdit_14->text().toDouble();
    double velHome = ui->lineEdit_15->text().toDouble();

    auto message = std_msgs::msg::Float64MultiArray();
    message.data = {velLimit, velHome};
    publishVelHomeToRos->publish(message);
    
    QProcess *process = new QProcess(this);
    
    QString program = "/bin/bash";
    QString wsPath = "/home/theanh/ros2_ws1";
    QString scriptPath = wsPath + "/src/control_one_motor_pkg_ex/scripts/control_motor.py";

    QString command = QString("source /opt/ros/humble/setup.bash && "
                              "source %1/install/setup.bash && "
                              "python3 %2").arg(wsPath).arg(scriptPath);

    QStringList arguments;
    arguments << "-c" << command; 

    process->start(program, arguments);

    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), 
        process, &QProcess::deleteLater);

    if (!process->waitForStarted()) {
        qDebug() << "Lỗi: Không thể khởi động script Python!";
    } else {
        qDebug() << "dang chay homing...";
    }
}

