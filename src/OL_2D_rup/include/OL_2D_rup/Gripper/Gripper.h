#ifndef OL_2D_RUP_GRIPPER_H
#define OL_2D_RUP_GRIPPER_H

#include <armadillo>
#include <vector>
#include <OL_2D_rup/utils.h>
#include <param_lib/param_lib.h>
#include <ros/package.h>

class Gripper
{

public:
  Gripper();

  virtual void init() = 0;

  virtual void move() = 0;
  virtual void close() = 0;

  virtual bool stop() = 0;
  virtual void setStopFlag() = 0;
  virtual void resetStopFlag() = 0;
  bool stop_flag;
  
  virtual void shutdown() = 0;

  arma::Col<double> move_pos;
  arma::Col<double> close_force_thres;

protected:

};

#endif // OL_2D_RUP_GRIPPER_H
