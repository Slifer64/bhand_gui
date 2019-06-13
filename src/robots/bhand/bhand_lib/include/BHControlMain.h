////////////////////
//  Barrett Hand  //
////////////////////

#ifndef BARRETT_HAND_64_H
#define BARRETT_HAND_64_H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <deque>
#include <stdexcept>
#include <sstream>
#include "BHand.h"
#include "BHandAppHelper.h" // for connection menu


class BarrettHand
{
public:
    BarrettHand();
    BarrettHand(const std::string &_hand_type);
    ~BarrettHand();

    void initialize(const std::string &handType, bool rt_control_enable =  false);

    void waitNextCycle();

    double getControlCycle() const;

    void terminate();

    bool is_initialized() const;
    bool is_RT_control_enabled() const;

    void enable_RT_control();

    void printErrorMessage(int err_id) const;
    void printErrorMessage(std::string &buf, int err_id) const;

    double getFingerPosition(int i); // const;
    int getFingerVelocity(int i); // const;
    double getFingerForce(int i); // const;
    int getMaxTemp(); // const;

    int setFingerVelocity(int vel, int i);
    int setFingerPosition(double pos, int i);
    int setFingerTorque(int torque, int i);
    int setFingerGain(int gain, int i);

    double get_max_finger_pos(int i);
    double get_min_finger_pos(int i);

    int command(const char *send, char *receive = 0);
    void set_param(const char *motor, const char *propertyName, int value);
    void get_param(const char *motor, const char *propertyName, int *result);

    double getNewtonMetersFromSgValue(int sg_value) const;

	BHand  bh;           // Handles all hand communication
private:

	void set_max_min_pos();

  // char   buf[200];     // Buffer for reading back hand parameters
  // int    value;        // Some commands use an int* instead of a buffer to relay information
  // int    err;       // Return value (error) of all BHand calls

  double ctrl_cycle;

  bool initialized;
  bool RT_control_enabled;

  double max_pos[4];
  double min_pos[4];

  std::vector<std::pair<double, double>> joint_limits;
  std::vector<std::pair<int, int>> joint_limits_ticks;

  std::string hand_type;

	bool control_velocity_flag ;	            // LCV: Loop Control Velocity Flag
	bool control_propgain_flag;	                // LCPG: Loop Control Proportional Gain Flag
	bool control_torque_flag;	                // LCT: Loop Control Torque Flag
	bool control_position_flag;					// LCP: Loop Control Position Flag
	bool feedback_velocity_flag;	            // LFV: Loop Feedback Velocity Flag
	bool feedback_strain_flag;	                // LFS: Loop Feedback Stain Flag
	bool feedback_position_flag;	            // LFAP: Loop Feedback Absolute Position Flag
	bool feedback_deltapos_flag;	            // LFDP: Loop Feedback Delta Position Flag
	bool feedback_breakaway_position_flag;	    // LFBP: Loop Feedback Breakaway Position Flag
	bool feedback_analog_input_flag;	        // LFAIN: Loop Feedback Analog Input Flag
	bool feedback_delta_position_discard_flag;	// LFDPD: Loop Feedback Delta Position Discard Flag
	bool feedback_temperature;	                // LFT: Loop Feedback Temperature Flag

	int control_velocity_coefficient;           // LCVC: Loop Control Velocity Coefficient
	int feedback_velocity_coefficient;          // LFVC: Loop Feedback Velocity Coefficient
	int feedback_delta_position_coefficient;    // LFDPC: Loop Feedback Delta Position Coefficient

    void check_finger_index(int i) const;

    double ticks2rad(int i, int ticks) const;
    int rads2ticks(int i, double rad) const;
};


#endif
