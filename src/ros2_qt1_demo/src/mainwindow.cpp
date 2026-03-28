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

    // khoi tao thread nhan du lieu trang thai motor tu joint states
    displayStatusMotor = new DisplayStatusMotor();
    displayStatusMotorThread = new QThread(this);

    displayStatusMotor->moveToThread(displayStatusMotorThread);
    connect(displayStatusMotorThread, &QThread::started, displayStatusMotor, &DisplayStatusMotor::start);
    displayStatusMotorThread->start();

    connect(displayStatusMotor, &DisplayStatusMotor::jointDataReceive, this, [this](int index, double position, double velocity){
        if(index == 0) {
            ui->lineEdit_14->setText(QString::number(position));
            ui->lineEdit_20->setText(QString::number(velocity));
        }

        if(index == 1) {
            ui->lineEdit_15->setText(QString::number(position));
            ui->lineEdit_21->setText(QString::number(velocity));
        }

        if(index == 2) {
            ui->lineEdit_16->setText(QString::number(position));
            ui->lineEdit_22->setText(QString::number(velocity));
        }

        if(index == 3) {
            ui->lineEdit_17->setText(QString::number(position));
            ui->lineEdit_23->setText(QString::number(velocity));
        }

        if(index == 4) {
            ui->lineEdit_18->setText(QString::number(position));
            ui->lineEdit_24->setText(QString::number(velocity));
        }

        if(index == 5) {
            ui->lineEdit_19->setText(QString::number(position));
            ui->lineEdit_25->setText(QString::number(velocity));
        }
    });

    connect(displayStatusMotor, &DisplayStatusMotor::jointDataReceiveDynamic, this, [this](int index, int status, int mode){
        QString nameMode = (mode == 9) ? "Vel Mode" : 
                           (mode == 8) ? "Pos Mode" : 
                           (mode == 6) ? "Home Mode"   : "Unknown";

        bool servoOn = (status & 0x0004); // Bit 2
        bool fault   = (status & 0x0008); // Bit 3

        QLineEdit* modeFields[] = { ui->lineEdit_26, ui->lineEdit_27, ui->lineEdit_28, 
                                    ui->lineEdit_29, ui->lineEdit_30, ui->lineEdit_31 };

        QPushButton* statusBtns[] = { ui->pushButton_10, ui->pushButton_12, ui->pushButton_13, 
                                    ui->pushButton_14, ui->pushButton_15, ui->pushButton_16};

        if (index >= 0 && index < 6) {
            modeFields[index]->setText(nameMode);

            QPushButton* btn = statusBtns[index];
            if (fault) {
                btn->setStyleSheet("background-color: red;");
            } else if (servoOn) {
                btn->setStyleSheet("background-color: green;");
            } else {

                btn->setStyleSheet("background-color: white;");
            }
        }
    });

    // thiet lap publisher de gui lenh vel homing tu GUI sang ROS
    nodeMode = rclcpp::Node::make_shared("gui_mode_publisher");
    publisherMode = nodeMode->create_publisher<std_msgs::msg::String>("motor_command", 10);

    connect(ui->pushButton_2, &QPushButton::clicked, this, &MainWindow::enableMotor);
    connect(ui->pushButton_5, &QPushButton::clicked, this, &MainWindow::disableMotor);
    connect(ui->pushButton_6, &QPushButton::clicked, this, &MainWindow::resetFaultError);


    // send vel homing to ros from GUI
    // nodePubVelHome = rclcpp::Node::make_shared("gui_homing_publisher");
    // publishVelHomeToRos = nodePubVelHome->create_publisher<std_msgs::msg::Float64MultiArray>("homing_manager_topic", 10);
    connect(ui->pushButton_11, &QPushButton::clicked, this, &MainWindow::sendVelHomeToRos);


    // send position and rotate angel
    nodeMoveToPositions = rclcpp::Node::make_shared("target_pose_node");
    publishPose = nodeMoveToPositions->create_publisher<std_msgs::msg::Float64MultiArray>("pos_and_rotate_angle_topic", 10);
    connect(ui->pushButton_21, &QPushButton::clicked, this, &MainWindow::sendPose);


    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onButtonClicked);

    //Velocity Mode
    connect(ui->pushButton_8, &QPushButton::clicked, this, [this](){ 
        uint8_t targetMode = 9; 
        
        qDebug() << "chuyen tat ca motor sang mode" << targetMode;
        
        for(int slave = 0; slave < 6; slave++) {
            this->modeOfOperation(targetMode, slave);
            QThread::msleep(80);
        }
    });

    // Position Mode
    connect(ui->pushButton_7, &QPushButton::clicked, this, [this](){ 
        uint8_t targetMode = 8; 
        
        qDebug() << "chuyen tat ca motor sang mode" << targetMode;
        
        for(int slave = 0; slave < 6; slave++) {
            this->modeOfOperation(targetMode, slave);
            QThread::msleep(80);
        }
    });

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

void MainWindow::enableMotor()
{
    QProcess *process = new QProcess(this);

    QString program = "/bin/bash";
    QString wsPath = "/home/theanh/ros2_ws1";
    QString scriptPath = wsPath + "/src/control_one_motor_pkg_ex/scripts/sync_axis.py";


    QString command = QString("python3 %1").arg(scriptPath);

    QStringList arguments;
    arguments << "-c" << command; 

    process->start(program, arguments);

    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), [=](int exitCode) {
        if (exitCode == 0) {
            auto message = std_msgs::msg::String();
            message.data = "enable";
            publisherMode->publish(message);
            
            qDebug() << "Published 'enable' command to ROS";

        } else {
            qDebug() << " Can't enable motor because Sync Failed";
        }
        process->deleteLater();
    });

    // auto message = std_msgs::msg::String();
    // message.data = "enable";
    // publisherMode->publish(message);
}

void MainWindow::disableMotor()
{
    auto message = std_msgs::msg::String();
    message.data = "disable";
    publisherMode->publish(message);
    
    qDebug() << "Published 'disable' command to ROS";
}

void MainWindow::resetFaultError()
{
    auto message = std_msgs::msg::String();
    message.data = "fault_reset";
    publisherMode->publish(message);
    
    qDebug() << "Published 'fault_reset' command to ROS";
}


void MainWindow::modeOfOperation(uint8_t mode, uint8_t slave)
{
    QString wsPath = "/home/theanh/ros2_ws1";
    
    if (mode == 8) {
        QProcess *syncProcess = new QProcess(this);
        connect(syncProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), [=](int exitCode) {
            if (exitCode == 0) {
                executeModeSwitch(mode, slave, wsPath);
            } else {
                qDebug() << "Slave" << slave << "Sync Failed!";
            }
            syncProcess->deleteLater();
        });

        QString syncScript = wsPath + "/src/control_one_motor_pkg_ex/scripts/sync_axis.py";
        QString fullSyncCmd = QString("source /opt/ros/humble/setup.bash && source %1/install/setup.bash && python3 -u %2")
                              .arg(wsPath).arg(syncScript);
        
        syncProcess->start("/bin/bash", QStringList() << "-c" << fullSyncCmd);
    } 
    else {
        executeModeSwitch(mode, slave, wsPath);
    }
}

void MainWindow::executeModeSwitch(uint8_t mode, uint8_t slave, QString wsPath)
{
    QProcess *process = new QProcess(this);
    
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), process, &QObject::deleteLater);

    QString command = QString(
        "source /opt/ros/humble/setup.bash && source %1/install/setup.bash && "
        "ros2 service call /ethercat_manager/set_sdo ethercat_msgs/srv/SetSdo "
        "\"{master_id: 0, slave_position: %2, sdo_index: 0x6060, sdo_subindex: 0, sdo_data_type: 'int8', sdo_value: '%3'}\""
    ).arg(wsPath).arg(slave).arg(mode);

    process->start("/bin/bash", QStringList() << "-c" << command);
    
    qDebug() << "chuyen mode" << mode << "cho slave:" << slave;
}

///////////////////////// dang loi ///////////////////////////

// void MainWindow::modeOfOperation(uint8_t mode)
// {
//     QString wsPath = "/home/theanh/ros2_ws1";
    
//     if (mode == 8) {
//         QProcess *syncProcess = new QProcess(this);
        
//         QString syncScript = wsPath + "/src/control_one_motor_pkg_ex/scripts/sync_axis.py";
//         QString fullSyncCmd = "source /opt/ros/humble/setup.bash && source " + wsPath + "/install/setup.bash && python3 -u " + syncScript;

//         qDebug() << "Step 1: Running Sync Script with Source...";

//         syncProcess->start("/bin/bash", QStringList() << "-c" << fullSyncCmd);

//         connect(syncProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), [=](int exitCode) {
//             if (exitCode == 0) {
//                 qDebug() << "Step 1 Success: Sync finished.";
//                 executeModeSwitch(mode, wsPath);
//             } else {
//                 QString errorOutput = syncProcess->readAllStandardError();
//                 qDebug() << "Step 1 Failed! Error: " << errorOutput;
//             }
//             syncProcess->deleteLater();
//         });
//     } 
//     else {
//         executeModeSwitch(mode, wsPath);
//     }
    
// }

// void MainWindow::executeModeSwitch(uint8_t mode, QString wsPath)
// {
//     QProcess *process = new QProcess(this);
    
//     QString activateController = (mode == 8) ? "trajectory_controller" : "velocity_controller";
//     QString deactivateController = (mode == 8) ? "velocity_controller" : "trajectory_controller";

//     QString command = "source /opt/ros/humble/setup.bash && source " + wsPath + "/install/setup.bash && ";

//     // command += "ros2 control set_controller_state " + deactivateController + " inactive && ";

//     // command += "sleep 1.0 && ";

//     command += "for i in {0..5}; do "
//                "ros2 service call /ethercat_manager/set_sdo ethercat_msgs/srv/SetSdo "
//                "\"{master_id: 0, slave_position: $i, sdo_index: 0x6060, sdo_subindex: 0, sdo_data_type: 'int8', sdo_value: '" + QString::number(mode) + "'}\"; "
//                "done && ";

//     // command += "sleep 1.0 && ";

//     // command += "ros2 control set_controller_state " + activateController + " active";

//     qDebug() << "Executing Multi-Slave Switch Mode:" << mode;
    
//     process->start("/bin/bash", QStringList() << "-c" << command);
    
//     connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), [process](int exitCode){
//         if (exitCode == 0) qDebug() << "All slaves switched mode successfully.";
//         else qDebug() << "Multi-slave switch FAILED.";
//         process->deleteLater();
//     });
// }

/////////////////////////////////////////////////////////////////////


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
    // double velLimit = ui->lineEdit_14->text().toDouble();
    // double velHome = ui->lineEdit_15->text().toDouble();

    // auto message = std_msgs::msg::Float64MultiArray();
    // message.data = {velLimit, velHome};
    // publishVelHomeToRos->publish(message);
    
    QProcess *process = new QProcess(this);
    
    QString program = "/bin/bash";
    QString wsPath = "/home/theanh/ros2_ws1";
    QString scriptPath = wsPath + "/src/control_one_motor_pkg_ex/scripts/home_motor.py";

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

void MainWindow::sendPose()
{
    float X = ui->doubleSpinBox->text().toDouble();
    float Y = ui->doubleSpinBox_2->text().toDouble();
    float Z = ui->doubleSpinBox_3->text().toDouble();
    float Roll = ui->doubleSpinBox_4->text().toDouble();
    float Pitch = ui->doubleSpinBox_5->text().toDouble();
    float Yaw = ui->doubleSpinBox_6->text().toDouble();

    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {X, Y, Z, Roll, Pitch, Yaw};
    publishPose->publish(msg);
}

