#include "gripper_mainwindow.h"
#include "ui_gripper_mainwindow.h"

Gripper_MainWindow::Gripper_MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Gripper_MainWindow)
{
    ui->setupUi(this);

    init();
}

Gripper_MainWindow::~Gripper_MainWindow()
{
    delete ui;
    if (receiveMsgs_thread.joinable()) receiveMsgs_thread.join();
}

void Gripper_MainWindow::init()
{
    this->setWindowTitle(QApplication::translate("Gripper", "Gripper", 0));

    run_gripper = false;
    init_gripper = false;
    move_gripper = false;
    close_gripper = false;
    open_gripper = false;

    gripper_move_pos.resize(3);
    gripper_torque_thres.resize(3);

    receiveMsgs_thread = std::thread(&Gripper_MainWindow::checkForReceivedMsgs, this);
}

void Gripper_MainWindow::checkForReceivedMsgs()
{
    while (true)
    {
        std::mutex mtx;
        std::unique_lock<std::mutex> lck(mtx);

        msg_receiv_cond.wait(lck);

        std::unique_lock<std::mutex> lck2(msg_mtx);
        switch (receiv_msg_type) {
          case Ui::MSG_TYPE::INFO:
            PRINT_INFO_MSG(receiv_msg.c_str());
            break;
          case Ui::MSG_TYPE::WARNING:
            PRINT_WARN_MSG(receiv_msg.c_str());
            break;
          case Ui::MSG_TYPE::ERROR:
            PRINT_ERR_MSG(receiv_msg.c_str());
            break;
        }
    }
}

void Gripper_MainWindow::PRINT_INFO_MSG(const std::string &msg)
{
  ui->msg_label->setText(("[INFO]: " + msg).c_str());
}

void Gripper_MainWindow::PRINT_WARN_MSG(const std::string &msg)
{
  ui->msg_label->setText(("[WARNING]: " + msg).c_str());
}

void Gripper_MainWindow::PRINT_ERR_MSG(const std::string &msg)
{
  ui->msg_label->setText(("[ERROR]: " + msg).c_str());
}

void Gripper_MainWindow::setMsg(const char *msg, Ui::MSG_TYPE msg_type)
{
  receiv_msg = msg;
  receiv_msg_type = msg_type;
  msg_receiv_cond.notify_one();
}

void Gripper_MainWindow::on_init_gripper_btn_clicked()
{
  std::unique_lock<std::mutex> lck(msg_mtx);

  init_gripper = true;

  notifyGripper();

  PRINT_INFO_MSG("Initializing gripper.");
}

void Gripper_MainWindow::on_move_gripper_btn_clicked()
{
  std::unique_lock<std::mutex> lck(msg_mtx);

  gripper_move_pos[0] = ui->finger1_pos_val->text().toDouble() * 3.14159/180;
  gripper_move_pos[1] = ui->finger2_pos_val->text().toDouble() * 3.14159/180;
  gripper_move_pos[2] = ui->finger3_pos_val->text().toDouble() * 3.14159/180;

  move_gripper = true;

  notifyGripper();

  PRINT_INFO_MSG("Moving the fingers of the gripper.");
}

void Gripper_MainWindow::on_close_gripper_btn_clicked()
{
  std::unique_lock<std::mutex> lck(msg_mtx);

  gripper_torque_thres[0] = ui->finger1_torque_val->text().toDouble();
  gripper_torque_thres[1] = ui->finger2_torque_val->text().toDouble();
  gripper_torque_thres[2] = ui->finger3_torque_val->text().toDouble();

  close_gripper = true;

  notifyGripper();

  PRINT_INFO_MSG("Closing the fingers of the gripper.");
}

void Gripper_MainWindow::on_stop_gripper_btn_clicked()
{
  std::unique_lock<std::mutex> lck(msg_mtx);

  *stop_gripper = true;

  notifyGripper();

  PRINT_INFO_MSG("Stopping the gripper.");
}

void Gripper_MainWindow::on_open_gripper_btn_clicked()
{
    std::unique_lock<std::mutex> lck(msg_mtx);

    open_gripper = true;

    notifyGripper();

    PRINT_INFO_MSG("Openning the gripper.");
}

void Gripper_MainWindow::notifyGripper()
{
  std::unique_lock<std::mutex> lck;
  run_gripper = true;
  gripper_run_cond.notify_one();
}

void Gripper_MainWindow::finalize()
{
  *stop_gripper = true;
  notifyGripper();
}
