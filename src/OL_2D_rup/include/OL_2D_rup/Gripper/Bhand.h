#ifndef OL_2D_RUP_BHAND_GRIPPER_H
#define OL_2D_RUP_BHAND_GRIPPER_H

#include <cstdlib>
#include <memory>
#include <BHControlMain.h>

#include <OL_2D_rup/Gripper/Gripper.h>

class Bhand : public Gripper
{

public:
  Bhand();

  void init();

  void move();

  void close();

  bool stop();
  void setStopFlag();
  void resetStopFlag();

  void shutdown();

private:
  std::shared_ptr<BarrettHand> hand;
};

#endif // OL_2D_RUP_BHAND_GRIPPER_H
