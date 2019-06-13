/**
 * Copyright (C) 2017 as64_
 */

#include <ros/ros.h>
#include <ros/package.h>

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <OL_2D_rup/Gripper/Gripper.h>
#include <OL_2D_rup/Gripper/Bhand.h>
#include <gripper_mainwindow.h>
#include <QApplication>

class GripperController
{
private:
  std::unique_ptr<Gripper_MainWindow> gui_obj;
  std::unique_ptr<Gripper> gripper;

  std::thread gripper_thread;
  std::thread gui_thread;

  bool stop_gripper_thread;

  bool run_gripper;
  std::mutex gripper_run_mtx;
  std::condition_variable gripper_run_cond;

  bool gui_init;
  bool gripper_init;

public:
  GripperController()
  {
    gui_init = false;
    gripper_init = false;

    stop_gripper_thread = false;

    gripper.reset(new Bhand());

    gripper_thread = std::thread(&GripperController::gripperThreadFun, this);
    gui_thread = std::thread(&GripperController::guiThreadFun, this);

    gui_thread.join();
    gripper_thread.join();

  }

  std::vector<double> getGripperMovePos() const
  {
    return gui_obj->gripper_move_pos;
  }

  std::vector<double> getGripperForceThres() const
  {
    return gui_obj->gripper_torque_thres;
  }

  void gripperWait()
  {
    std::unique_lock<std::mutex> lck(gui_obj->gripper_run_mtx);
    while (!gui_obj->run_gripper) gui_obj->gripper_run_cond.wait(lck);
    gui_obj->run_gripper = false;
  }

  bool initGripper() const
  {
    return gui_obj->init_gripper;
  }

  void resetInitGripper()
  {
    gui_obj->init_gripper = false;
  }

  bool moveGripper() const
  {
    return gui_obj->move_gripper;
  }

  void resetMoveGripper()
  {
    gui_obj->move_gripper = false;
  }

  bool openGripper() const
  {
    return gui_obj->open_gripper;
  }

  void resetOpenGripper()
  {
    gui_obj->open_gripper = false;
  }

  bool closeGripper() const
  {
    return gui_obj->close_gripper;
  }

  void resetCloseGripper()
  {
    gui_obj->close_gripper = false;
  }

  bool terminateGripper() const
  {
    return false; //gui_obj->stop_gripper;
  }

  void resetTerminateGripper()
  {
    *(gui_obj->stop_gripper )= false;
  }

  int guiThreadFun()
  {
    int argc = 0;
    char **argv = NULL;

    QApplication a(argc, argv);
    gui_obj.reset(new Gripper_MainWindow);
    gui_init = true;
    gui_obj->show();

    gui_obj->setMsg("Initializing gripper...", Ui::MSG_TYPE::INFO);

    while (!gripper_init);
    gui_obj->stop_gripper = &(gripper->stop_flag);

    //start_cond.notify_one();

    int ret_val = a.exec();

    gui_obj->finalize();

    stop_gripper_thread = true;

    return ret_val;
  }

  int gripperThreadFun()
  {
    gripper->init();

    gripper_init = true;

    while (!gui_init) std::this_thread::sleep_for(std::chrono::milliseconds(1));

    while (!stop_gripper_thread)
    {
      gripperWait();

      if (this->initGripper())
      {
        gripper->init();
        this->resetInitGripper();
      }

      if (this->openGripper())
      {
        gripper->move_pos = arma::vec().zeros(this->getGripperMovePos().size());
        gripper->move();
        this->resetOpenGripper();
      }

      if (this->moveGripper())
      {
        gripper->move_pos = this->getGripperMovePos();
        gripper->move();
        this->resetMoveGripper();
      }

      if (this->closeGripper())
      {
        gripper->close_force_thres = this->getGripperForceThres();
        gripper->close();
        this->resetCloseGripper();
      }

      if (this->terminateGripper())
      {
        gripper->shutdown();
        this->resetTerminateGripper();
        break;
      }
    }
  }

};

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "gripper_node");

  GripperController grip_ctrl;

  return 0;
}
