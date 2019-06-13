////////////////////
//  Barrett Hand  //
////////////////////

#include "BHControlMain.h"


BarrettHand::BarrettHand()
{
    set_max_min_pos();
}

BarrettHand::BarrettHand(const std::string &_hand_type)
{
	initialized = RT_control_enabled = false;
	hand_type = _hand_type;

  ctrl_cycle = 0.005; // control cycle 5 ms

	set_max_min_pos();

}

double BarrettHand::getControlCycle() const
{
  return ctrl_cycle;
}

double BarrettHand::ticks2rad(int i, int ticks) const
{
  return joint_limits[i].first + (ticks - joint_limits_ticks[i].first)*(joint_limits[i].second - joint_limits[i].first)/(double)(joint_limits_ticks[i].second - joint_limits_ticks[i].first);
}

int BarrettHand::rads2ticks(int i, double rads) const
{
  return 0.5 + joint_limits_ticks[i].first + (rads - joint_limits[i].first)*(joint_limits_ticks[i].second - joint_limits_ticks[i].first)/(joint_limits[i].second - joint_limits[i].first);
}

double BarrettHand::getNewtonMetersFromSgValue(int sg_value) const
{
  double max_tip_torque = 40 * 0.06;  // 5kg of max tip force with 6cm distal link size
  double min_tip_torque = -40 * 0.06;  // -5kg of min tip force with 6cm distal link size
  double min_sg = 0;
  double max_sg = 255;

  double sg_perc = (static_cast<double>(sg_value) + min_sg) / (max_sg - min_sg);
  return sg_perc * (max_tip_torque - min_tip_torque) + min_tip_torque ;
}

void BarrettHand::set_max_min_pos()
{
  joint_limits_ticks.push_back(std::pair<int, int>(0, 0));
  joint_limits_ticks.push_back(std::pair<int, int>(6, 195100)); // finger 1
  joint_limits_ticks.push_back(std::pair<int, int>(3, 195250)); // finger 2
  joint_limits_ticks.push_back(std::pair<int, int>(12, 194900)); // finger 3

  joint_limits.push_back(std::pair<double, double>(0.0, 0.0));
  joint_limits.push_back(std::pair<double, double>(0.0, 2.44)); // finger 1
  joint_limits.push_back(std::pair<double, double>(0.0, 2.44)); // finger 2
  joint_limits.push_back(std::pair<double, double>(0.0, 2.44)); // finger 3

  max_pos[0] = 10000;
  max_pos[1] = 195100;
  max_pos[2] = 195250;
  max_pos[3] = 194900;

  min_pos[0] = 0;
  min_pos[1] = 6;
  min_pos[2] = 3;
  min_pos[3] = 12;
}

void BarrettHand::set_param(const char *motor, const char *propertyName, int value)
{
	int err = bh.Set(motor, propertyName, value);

	if (err){
		if (err == -1){
		  printErrorMessage(err);
		  return;
		}
		std::string buf;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
}

void BarrettHand::get_param(const char *motor, const char *propertyName, int *result)
{
	int err = bh.Get(motor, propertyName, result);

	if (err){
		if (err == -1){
		  printErrorMessage(err);
		  return;
		}
		std::string buf;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
}

BarrettHand::~BarrettHand()
{
	this->terminate();
}

// Checks whether the referenced finger's value is between 1 and 3
void BarrettHand::check_finger_index(int i) const
{
	if (i<0 || i>3){
		std::ostringstream err_msg;
		err_msg << "ERROR in \"BarrettHand: valid finger indexes are \"0,1,2,3\".\n Given finger index (" << i << ") is invalid.";
		throw std::invalid_argument(err_msg.str());
	}
}

// Initializes the Barrett Hand.
void BarrettHand::initialize(const std::string &handType, bool rt_control_enable)
{
	int err; // Return value (error) of all BHand calls
	std::string buf; // buffer for the error message

	// Set hardware description before initialization
	// int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-262");
	// int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-280");
	this->hand_type = handType;
	int hwIndex = BHandHardware::getBHandHardwareIndex(hand_type.c_str());
	if (hwIndex < 0){
		throw std::runtime_error("\n\nThe API has not been compiled to include target hand.\n");
		// printf("\n\nThe API has not been compiled to include target hand.\n");
		// this->printErrorMessage();
		//return;
	}

	bh.setHardwareDesc(hwIndex);
	//bool use280Config = (strcmp(bh.getHardwareDesc()->getModelNumber(), "BH8-280") == 0);
	//printf("\nuse280Config = %d\n", use280Config);
	if ((err = handInitWithMenu(&bh))){
		//this->printErrorMessage(err);
		// return;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
	//if (err = bh.InitSoftware(com_port, THREAD_PRIORITY_TIME_CRITICAL))
	//	Error();

	//printf("Initialization...");
	if ((err = bh.InitHand("123S"))){
		//this->printErrorMessage(err);
		//return;
		printErrorMessage(buf, err);
		throw std::runtime_error(buf);
	}
	//printf(" Done\n");

	initialized = true;

	if (rt_control_enable) enable_RT_control();
}

int BarrettHand::command(const char *send, char *receive)
{
	return bh.Command(send, receive);
}

// Stops the Barrett Hand and shuts down the motor.
void BarrettHand::terminate()
{
	bh.RTAbort();
	bh.StopMotor("123S");
	initialized = RT_control_enabled = false;
}

double BarrettHand::get_max_finger_pos(int i)
{
	check_finger_index(i);
	return max_pos[i];
}

double BarrettHand::get_min_finger_pos(int i)
{
	check_finger_index(i);
	return min_pos[i];
}


void BarrettHand::printErrorMessage(std::string &buf, int err_id) const
{
	const int buff_size = 100;
	char temp_buff[buff_size];
	snprintf(temp_buff, buff_size, "ERROR: %d\n%s\n", err_id, bh.ErrorMessage(err_id));

	buf = temp_buff;
}

void BarrettHand::printErrorMessage(int err_id) const
{
	std::string buf;
	printErrorMessage(buf, err_id);
	std::cerr << buf;
}

void BarrettHand::enable_RT_control()
{
	int err; // Return value (error) of all BHand calls

	/*if ((err = bh.RTSetFlags( "123", 1, 3, 0, 0, 0, 1, 1, 1, 0, 1,0,0,0,0 )))
		this->printErrorMessage(err);
	sprintf(buf, "%s", "");

	bh.RTStart( "123" );
	bh.RTUpdate();

	RT_control_enabled = true;

	feedback_strain_flag = feedback_position_flag = control_velocity_flag = true;
	*/

	char motor[] = "123S";

	control_velocity_flag                = true;   // LCV   Loop Control Velocity Flag
	control_propgain_flag                = false;  // LCPG  Loop Control Proportional Gain Flag
	control_torque_flag                  = false;  // LCT   Loop Control Torque Flag
	feedback_velocity_flag               = true;   // LFV   Loop Feedback Velocity Flag
	feedback_strain_flag                 = true;   // LFS   Loop Feedback Stain Flag
	feedback_position_flag               = true;   // LFAP  Loop Feedback Absolute Position Flag
	feedback_deltapos_flag               = false;  // LFDP  Loop Feedback Delta Position Flag
	feedback_breakaway_position_flag     = false;  // LFBP  Loop Feedback Breakaway Position Flag
	feedback_analog_input_flag           = false;  // LFAIN Loop Feedback Analog Input Flag
	feedback_delta_position_discard_flag = false;  // LFDPD Loop Feedback Delta Position Discard Flag
	feedback_temperature                 = true;   // LFT   Loop Feedback Temperature Flag

	control_position_flag                = false;

	control_velocity_coefficient        = 3;  // LCVC  Loop Control Velocity Coefficient
	feedback_velocity_coefficient       = 1;  // LFVC  Loop Feedback Velocity Coefficient
	feedback_delta_position_coefficient = 1;  // LFDPC Loop Feedback Delta Position Coefficient

	if ((err = bh.RTSetFlags(motor,
                control_velocity_flag, control_velocity_coefficient, control_propgain_flag,
                control_torque_flag, feedback_velocity_flag, feedback_velocity_coefficient,
                feedback_strain_flag, feedback_position_flag, feedback_deltapos_flag,
                feedback_delta_position_coefficient, feedback_breakaway_position_flag,
                feedback_analog_input_flag, feedback_delta_position_discard_flag,
                feedback_temperature ))){
					std::string buf;
					printErrorMessage(buf, err);
					throw std::runtime_error(buf);
					// this->printErrorMessage(err);
	}

	//bh.RTStart( "123S" , BHMotorTSTOPProtect);
	bh.RTStart( "123S" , BHMotorTorqueLimitProtect);
	bh.RTUpdate();

	RT_control_enabled = true;
}

bool BarrettHand::is_initialized() const
{
	return initialized;
}

bool BarrettHand::is_RT_control_enabled() const
{
	return RT_control_enabled;
}

// Updates the Barrett Hand.
// Must be called at every control cycle.
void BarrettHand::waitNextCycle()
{
	if (this->is_RT_control_enabled()) bh.RTUpdate();
}

// Gets RealTime absolute position feedback for the desired motor.
// The loop feedback absolute position (LFAP) flag must be set to receive absolute position feedback.
double BarrettHand::getFingerPosition(int i)
{
  // Check if the feedback position flag is set
  if (!feedback_position_flag)
    throw std::runtime_error("Error: BarrettHand: getFingerPosition: The loop feedback absolute position (LFAP) flag must be set to receive absolute position feedback.");

  if (i < 0)
  {
    // In case of negative value the convention is that we want the position of
    // every finger.
    return bh.RTGetPosition('G');
  }
  else if (i == 0)
  {
    // In case of zero ID the convention is that we want the position of the
    // spread joint.
    return bh.RTGetPosition('4');
  }

  // In any other case we check if the ID is one of the fingers and return this
  // value
  check_finger_index(i);
  return ticks2rad(i, bh.RTGetPosition(i + '0'));
}

// Gets RealTime velocity feedback for the desired motor.
// The loop feedback velocity (LFV) flag must be set to receive velocity feedback.
int BarrettHand::getFingerVelocity(int i)
{
	check_finger_index(i);

	if (!feedback_velocity_flag)
		throw std::runtime_error("Error: BarrettHand: getFingerVelocity: The loop feedback velocity (LFV) flag must be set to receive velocity feedback.");

	return bh.RTGetVelocity(i + '0');
}

// Gets RealTime strain gauge feedback for the desired motor.
// The loop feedback strain (LFS) flag must be set to receive strain gauge feedback.
double BarrettHand::getFingerForce(int i)
{
  // Check if the feedback position flag is set
  if (!feedback_strain_flag)
    throw std::runtime_error("Error: BarrettHand: getFingerForce: The loop feedback strain (LFS) flag must be set to receive strain gauge feedback.");

  if (i < 0)
  {
    // In case of negative value the convention is that we want the position of
    // every finger.
    return bh.RTGetStrain('G');
  }
  else if (i == 0)
  {
    // In case of zero ID the convention is that we want the position of the
    // spread joint.
    return bh.RTGetStrain('S');
  }

  // In any other case we check if the ID is one of the fingers and return this
  // value
  check_finger_index(i);
  return getNewtonMetersFromSgValue(bh.RTGetStrain(i + '0'));
}

// Sets RealTime control velocity reference for the desired motor.
int BarrettHand::setFingerVelocity(int vel, int i)
{
  // Check if the feedback position flag is set
  if (!control_velocity_flag)
		throw std::runtime_error("Error: BarrettHand: setFingerVelocity: The loop control velocity (LCV) flag must be set to send velocity references to the hand.");

  if (i < 0)
  {
    // In case of negative value the convention is that we send velocity to every
    // every finger.
    return bh.RTSetVelocity('G', vel);
  }
  else if (i == 0)
  {
    // In case of zero ID the convention is that we send velocity to the
    // spread joint.
    return bh.RTSetVelocity('4', vel);
  }

  // In any other case we check if the ID is one of the fingers so command the
  // velocity accordingly
  check_finger_index(i);
  return bh.RTSetVelocity(i + '0', vel);
}

// Sets RealTime position reference for the desired motor.
// RealTime position control is only possible with the 280 hand.
// The loop control position (LCP) flag needs to be set to send desired position references to the hand.
int BarrettHand::setFingerPosition(double pos, int i)
{
  // Check if the feedback position flag is set
  if (!control_position_flag)
		throw std::runtime_error("Error: BarrettHand: setFingerPosition: The loop control position (LCP) flag must be set to send position references to the hand.");

  if (i < 0)
  {
    // In case of negative value the convention is that we send velocity to every
    // every finger.
    return bh.RTSetPosition('G', pos);
  }
  else if (i == 0)
  {
    // In case of zero ID the convention is that we send velocity to the
    // spread joint.
    return bh.RTSetPosition('4', pos);
  }

  // In any other case we check if the ID is one of the fingers so command the
  // velocity accordingly
  check_finger_index(i);
  return bh.RTSetPosition(i + '0', pos);
}

// Sets RealTime control proportional gain for the desired motor.
int BarrettHand::setFingerGain(int gain, int i)
{
	check_finger_index(i);

	return bh.RTSetGain(i + '0', gain);
}

// Sets RealTime control torque for the desired motor.
// The loop control torque (LCT) flag needs to be set to send 16-bit torque references to the hand.
int BarrettHand::setFingerTorque(int torque, int i)
{
	check_finger_index(i);

	if (!control_torque_flag)
		throw std::runtime_error("Error: BarrettHand: setFingerTorque: The loop control torque (LCT) flag needs to be set to send 16-bit torque references to the hand.");

	return bh.RTSetTorque(i + '0', torque);
}

int BarrettHand::getMaxTemp()
{
  return bh.RTGetTemp();
}
