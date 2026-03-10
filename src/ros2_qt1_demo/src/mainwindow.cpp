#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onButtonClicked()
{
    qDebug() << "Button was clicked!";
}

// void MainWindow:::sendSdoCommand(uint16_t value) {
//     // Tạo client kết nối với service
//     auto client = ros_node->create_client<ethercat_msgs::srv::SetSdo>("/ethercat_manager/set_sdo");

//     if (!client->wait_for_service(std::chrono::seconds(1))) {
//         RCLCPP_ERROR(ros_node->get_logger(), "Service not available");
//         return;
//     }

//     auto request = std::make_shared<ethercat_msgs::srv::SetSdo::Request>();
//     request->master_id = 0;
//     request->slave_position = 0;
//     request->sdo_index = 0x6040;      // Controlword
//     request->sdo_subindex = 0;
//     request->sdo_data_type = "uint16"; 
//     request->sdo_value = std::to_string(value);

//     auto result = client->async_send_request(request, 
//         [](rclcpp::Client<ethercat_msgs::srv::SetSdo>::SharedFuture future) {
//             auto response = future.get();
//             if (response->success) {
//                 RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SDO Success: %s", response->sdo_return_message.c_str());
//             }
//         });
// }

// void MainWindow::on_btn_motor_on_clicked() {
//     motor_manager->sendSdoCommand(6); 
    
//     QTimer::singleShot(100, [this](){
//         motor_manager->sendSdoCommand(15);
//     });
// }

// void MainWindow::on_btn_motor_off_clicked() {
//     motor_manager->sendSdoCommand(6);
// }