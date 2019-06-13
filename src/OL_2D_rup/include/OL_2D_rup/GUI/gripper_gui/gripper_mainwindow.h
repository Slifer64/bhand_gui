#ifndef GRIPPER_MAINWINDOW_H
#define GRIPPER_MAINWINDOW_H

#include <QMainWindow>
#include <cstdlib>
#include <vector>
#include <thread>
#include <mutex>
#include <cstring>
#include <memory>
#include <condition_variable>

namespace Ui {

enum MSG_TYPE
{
  INFO,
  WARNING,
  ERROR
};

class Gripper_MainWindow;
}

class Gripper_MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit Gripper_MainWindow(QWidget *parent = 0);
    ~Gripper_MainWindow();

    void init();

    bool init_gripper;
    bool move_gripper;
    bool close_gripper;
    bool open_gripper;
    std::vector<double> gripper_move_pos;
    std::vector<double> gripper_torque_thres;

    std::mutex gripper_run_mtx;
    std::condition_variable gripper_run_cond;
    bool run_gripper;
    bool *stop_gripper;

    void setMsg(const char *msg, Ui::MSG_TYPE msg_type);

    void finalize();

private slots:
    void on_init_gripper_btn_clicked();

    void on_move_gripper_btn_clicked();

    void on_close_gripper_btn_clicked();

    void on_stop_gripper_btn_clicked();

    void on_open_gripper_btn_clicked();

private:
    Ui::Gripper_MainWindow *ui;

    std::mutex msg_mtx;

    void checkForReceivedMsgs();
    std::thread receiveMsgs_thread;
    std::string receiv_msg;
    Ui::MSG_TYPE receiv_msg_type;
    std::condition_variable msg_receiv_cond;

    void PRINT_INFO_MSG(const std::string &msg);
    void PRINT_WARN_MSG(const std::string &msg);
    void PRINT_ERR_MSG(const std::string &msg);

    void notifyGripper();

};


#endif // GRIPPER_MAINWINDOW_H
