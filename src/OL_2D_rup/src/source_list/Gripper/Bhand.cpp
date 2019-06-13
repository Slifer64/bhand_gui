#include <OL_2D_rup/Gripper/Bhand.h>

Bhand::Bhand()
{
  hand.reset(new BarrettHand("BH8-280"));

  init();
}

void Bhand::init()
{
  resetStopFlag();

  if (!hand->is_initialized())
  {
    hand->initialize("BH8-280", true);
    if (!hand->is_initialized())
    {
      throw std::runtime_error("Failed to initialize BarrettHand BH8-280.");
    }
  }

  if (!hand->is_RT_control_enabled())
  {
    hand->enable_RT_control();
    if (!hand->is_RT_control_enabled())
    {
      throw std::runtime_error("Failed to initialize BarrettHand BH8-280: RT control is not enabled.");
    }
  }

  hand->set_param("123S", "MODE", 0);

  // hand->set_param("123S", "TSTOP", 0);
  // hand->set_param("123S", "HSG", 10000);
  // hand->set_param("123S", "MODE", 4);

  // std::string path_to_config_file;
  // path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/Gripper_config.yml";
  // as64_::param_::Parser parser(path_to_config_file);
  //
  // if (!parser.getParam("move_pos", move_pos)) move_pos = {100, 100, 100};
  // if (!parser.getParam("close_force_thres", close_force_thres)) close_force_thres = {150, 150, 150};

}

bool Bhand::stop()
{
  return stop_flag;
}

void Bhand::setStopFlag()
{
  stop_flag = true;
}

void Bhand::resetStopFlag()
{
  stop_flag = false;
}

void Bhand::move()
{
  hand->set_param("123S", "TSTOP", 0);
  hand->set_param("123S", "HSG", 10000);
  hand->set_param("123S", "MODE", 4);

  arma::Col<double> temp(move_pos.size());
  arma::Col<double> vel(move_pos.size());

  arma::Col<double> y = {0, 0, 0};
  arma::Col<double> dy = {0, 0, 0};
  double T = 2;
  double K = std::pow(4.0/T, 2);
  double D = 2*std::sqrt(K);

  int fing_flag = 0;

  const char *hand_id[] = {"1", "2", "3"};

  resetStopFlag();

  while (!stop())
  {
    hand->waitNextCycle();

    double err = 0.0;
    for (int i=0;i<move_pos.size();i++)
    {
      temp(i) = hand->getFingerPosition(i+1);
      err = move_pos(i)-temp(i);

      if (std::fabs(err)<0.01)
      {
        hand->set_param(hand_id[i], "MODE", 0);
        fing_flag = fing_flag | (1<<i);
        vel(i) = 0.0;
      }
      else
      {
        int sign = 2*(err>=0)-1;
        vel(i) = sign*std::sqrt(std::fabs(err))*200;
        if (std::fabs(vel(i))>60) vel(i) = 60*sign;
        if (std::fabs(vel(i))<5) vel(i) = 5*sign;
        hand->setFingerVelocity(vel(i), i+1);
      }
    }

    // std::cerr << "temp = " << temp.t() << "\n";
    // std::cerr << "move_pos = " << move_pos.t() << "\n";
    // std::cerr << "vel = " << vel.t() << "\n";
    // std::cerr << "fing_flag = " << fing_flag << "\n";

    if (fing_flag == 7) break;

  }

  //for (int i=0;i<move_pos.size();i++) hand->setFingerVelocity(0, i+1);

  hand->set_param("123S", "MODE", 0);

    //hand->setFingerPosition(move_pos[i],i+1);
}

void Bhand::close()
{
  hand->set_param("123S", "TSTOP", 0);
  hand->set_param("123S", "HSG", 10000);
  hand->set_param("123S", "MODE", 4);

  arma::Col<double> temp(close_force_thres.size());
  arma::Col<double> vel(close_force_thres.size());

  int fing_flag = 0;

  const char *hand_id[] = {"1", "2", "3"};

  resetStopFlag();

  while (!stop())
  {
    hand->waitNextCycle();
    for (int i=0;i<move_pos.size();i++)
    {
      temp(i) = hand->getFingerForce(i+1);
      if (temp(i) > close_force_thres(i))
      {
        hand->set_param(hand_id[i], "MODE", 0);
        fing_flag = fing_flag | (1<<i);
        vel(i) = 0.0;
      }
      else
      {
        hand->setFingerVelocity(55, i+1);
      }
    }
    if (fing_flag == 7) break;

    std::cerr << "temp = " << temp.t() << "\n";
    std::cerr << "close_force_thres = " << close_force_thres.t() << "\n";
    std::cerr << "vel = " << vel.t() << "\n";
    std::cerr << "fing_flag = " << fing_flag << "\n";

  }

  hand->set_param("123S", "MODE", 0);
}

void Bhand::shutdown()
{
  hand->terminate();
}
