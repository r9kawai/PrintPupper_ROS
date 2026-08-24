#include <ros/ros.h>
#include <ros/package.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pigpiod_if2.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include "ComputeUnparallelLinkKnee.h"

std::vector<std::vector<float>> loadFloatArray(const std::string& filename);

// ----------------------------------------------------------------------------
class PpChampJointsHardware
{
public:
	const bool DBG_ON = false;
	const float PI = std::acos(-1.0);
	const float L45DEG_RAD = (PI / 4);
	const float R45DEG_RAD = -(PI / 4);

	enum LEGID {
		FL, FR, BL, BR, LEGS,
	};
	enum AXISID {
		HIP, LEG, KNE, AXIS,
	};
	const int JOINTS = LEGS * AXIS;
	const char* LEGSNAMES[LEGS] = { "FL", "FR", "BL", "BR" };	// legs 0, 1, 2, 3
	const char* AXISNAMES[AXIS] = { "Hip", "Leg", "Knee" };		// axis 0, 1, 2

	// Hip     機体水平=RAD0,左回転系（機体前から見た視点で
	// Leg     機体直下=RAD0,左回転系（機体左から見た視点で
	// Knee Leg一直線上=RAD0,左回転系（機体左から見た視点で
	float legs_axis_msg_pos_[LEGS][AXIS];	// champ からの指令位置
	float legs_axis_cmd_pos_[LEGS][AXIS];	// 変換された各サーボの指令位置

	// PrintPupper PWM servo
	// gpio pins assigen
	const int legs_axis_servo_gpio_pins_[LEGS][AXIS] = {
	/*             Hip,  Leg, Knee */
	/* FL */	{   17,   27,   22 },
	/* FR */	{   23,   24,   25 },
	/* BL */	{    5,    6,   19 },
	/* BR */	{   16,   20,   21 } };

	// rotate inverts assigen
	const float legs_axis_servo_inverts_[LEGS][AXIS] = {
	/*             Hip,  Leg, Knee */
	/* FL */	{    1,    1,    1 },
	/* FR */	{    1,   -1,   -1 },
	/* BL */	{   -1,    1,    1 },
	/* BR */	{   -1,   -1,   -1 } };

	// neutral offsets assigen rad
	float legs_axis_servo_neutrals_rad_[LEGS][AXIS] = {
	/*           Hip,        Leg,       Knee */
	/* FL */	{  0, L45DEG_RAD, R45DEG_RAD },
	/* FR */	{  0, L45DEG_RAD, R45DEG_RAD },
	/* BL */	{  0, L45DEG_RAD, R45DEG_RAD },
	/* BR */	{  0, L45DEG_RAD, R45DEG_RAD } };

	const int PWM_HZ = 100;
	const int PWM_USEC_RANGE = int(1000000) / PWM_HZ;
	const int PWM_USEC_MIN = 500;
	const int PWM_USEC_NEUTRAL = 1500;
	const int PWM_USEC_MAX = 2500;
	const float PWM_USEC_PER_RAD = (PWM_USEC_MAX - PWM_USEC_MIN) / PI;
	int pigpio_ = -1;

	ros::Subscriber sub_;
	ros::NodeHandle nh_;
	ComputeUnparallelLinkKnee culk_;

// ------------------------------------
	PpChampJointsHardware()
	{
		piGpiosPwmInit();

		// load ServoCalibration.conf to neutral offsets
		std::string package_path = ros::package::getPath("pp_champ_joints_hardware");
		std::string servo_calibration_conf_path = package_path + "/ServoCalibration.conf";
		auto conf_ar = loadFloatArray(servo_calibration_conf_path);
		bool exit_svconf = false;
		if(!conf_ar.empty()) {
			if (conf_ar.size() == 4 && conf_ar[0].size() == 3) {
				std::cout << "from " << servo_calibration_conf_path << std::endl;
				for(int leg = 0; leg < LEGS; leg++) {
					for(int axis = 0; axis < AXIS; axis++) {
						float deg_v = conf_ar[leg][axis];
						std::cout << std::fixed << std::setprecision(2) << std::showpos << deg_v << ", ";
					}
					std::cout << std::endl;
				}
				std::cout << std::endl;
				exit_svconf = true;
			}
		}
		if(!exit_svconf){
			std::cout << "Warning : cant load " << servo_calibration_conf_path << std::endl;
		}

		std::cout << "Current servo calibration values" << std::endl;
		for(int leg = 0; leg < LEGS; leg++) {
			for(int axis = 0; axis < AXIS; axis++) {
				if(exit_svconf){
					float deg_v = conf_ar[leg][axis];
					float rad_v_ = deg2rad(deg_v);
					rad_v_ *= legs_axis_servo_inverts_[leg][axis];
					legs_axis_servo_neutrals_rad_[leg][axis] -= rad_v_;
				}
				float rad_v = legs_axis_servo_neutrals_rad_[leg][axis];
				std::cout << std::fixed << std::setprecision(2) << std::showpos << rad2deg(rad_v) << ", ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;

		sub_ = nh_.subscribe("/joint_group_position_controller/command", 1,
			&PpChampJointsHardware::onSubscribeJointPosition, this);
		return;
	}

// ------------------------------------
	~PpChampJointsHardware()
	{
		std::cout << "~PpChampJointsHardware" << std::endl;
		sub_.shutdown();
		piGpiosPwmDone();
		return;
	}

// ------------------------------------
	void piGpiosPwmInit()
	{
		pigpio_ = pigpio_start(0, 0);
		if(pigpio_ < 0) {
			std::cout << "pigpio_start error!" << std::endl;
			return;
		}

		for(int leg = 0; leg < LEGS; leg++) {
			for(int axis = 0; axis < AXIS; axis++) {
				int pin = legs_axis_servo_gpio_pins_[leg][axis];
				set_PWM_frequency(pigpio_, pin, PWM_HZ);
				set_PWM_range(pigpio_, pin, PWM_USEC_RANGE);
				set_PWM_dutycycle(pigpio_, pin, PWM_USEC_NEUTRAL);
				ros::Duration(0.5).sleep();
			}
		}
		std::cout << "piGpiosPwmInit" << std::endl;
		return;
	}

// ------------------------------------
	void piGpiosPwmDone()
	{
		if(pigpio_ < 0) {
			return;
		}

		for(int leg = 0; leg < LEGS; leg++) {
			for(int axis = 0; axis < AXIS; axis++) {
				int pin = legs_axis_servo_gpio_pins_[leg][axis];
				set_PWM_dutycycle(pigpio_, pin, PWM_USEC_NEUTRAL);
				ros::Duration(0.5).sleep();
			}
		}
		/*
		for(int leg = 0; leg < LEGS; leg++) {
			for(int axis = 0; axis < AXIS; axis++) {
				int pin = legs_axis_servo_gpio_pins_[leg][axis];
				set_PWM_dutycycle(pigpio_, pin, 0);
			}
		}
		*/
		pigpio_stop(pigpio_);
		std::cout << "piGpiosPwmDone" << std::endl;
		return;
	}

// ------------------------------------
	void onSubscribeJointPosition(const trajectory_msgs::JointTrajectory::ConstPtr& _msg)
	{
		if (_msg->points.empty()) return;
		const auto& msgnames = _msg->joint_names;
		const auto& msgpos   = _msg->points[0].positions;
		const int poss = std::min(msgnames.size(), msgpos.size());
		if(poss != JOINTS) return;

		int msgpos_i = 0;
		bool change = false;
		for(int leg = 0; leg < LEGS; leg++) {
			for(int axis = 0; axis < AXIS; axis++) {
				float val = floor(msgpos[msgpos_i]*10000)/10000;
				if(legs_axis_msg_pos_[leg][axis] != val) {
					change = true;
				}
				legs_axis_msg_pos_[leg][axis] = val;
				msgpos_i++;
			}
		}

		culk_.dbgOn = false;
		if(change && DBG_ON) {
			culk_.dbgOn = true;
			std::cout << std::endl;
			axis_msg_pos_PrintDbg("msg_pos", legs_axis_msg_pos_);
		}
		extrapolationUnparallelActuatorPositions();
		if(change && DBG_ON) {
			axis_msg_pos_PrintDbg("cmd_pos", legs_axis_cmd_pos_);
		}

		sendLegsAxisPosToPwmToServos();
		return;
	}

// ------------------------------------
	void axis_msg_pos_PrintDbg(const char *printHead, float legs_axis[LEGS][AXIS])
	{
		std::cout << printHead << " | ";
		for(int leg = 0; leg < LEGS; leg++) {
			std::cout << LEGSNAMES[leg] << " ";
			for(int axis = 0; axis < AXIS; axis++) {
				float val = rad2deg(legs_axis[leg][axis]);
				std::cout << std::internal << std::fixed << std::setprecision(1) << std::setw(6) << std::right << val << ", ";
			}
			std::cout << "| ";
		}
		std::cout << std::endl;
		return;
	}

// ------------------------------------
	void sendLegsAxisPosToPwmToServos()
	{
		for(int leg = 0; leg < LEGS; leg++) {
			for(int axis = 0; axis < AXIS; axis++) {
				sendPosToPwmToServo(leg, axis);
			}
		}
		return;
	}

// ------------------------------------
	void sendPosToPwmToServo(int _leg, int _axis)
	{
		int pin = legs_axis_servo_gpio_pins_[_leg][_axis];
		float pos_rad = legs_axis_cmd_pos_[_leg][_axis];
		float pos_rad_i_n = posRad_InvertAndNeutral(pos_rad, _leg, _axis);
		int send_duty = posRadDutycycle(pos_rad_i_n);
		set_PWM_dutycycle(pigpio_, pin, send_duty);
		return;
	}

// ------------------------------------
	float posRad_InvertAndNeutral(float _pos_rad, int _leg, int _axis)
	{
		float inv = legs_axis_servo_inverts_[_leg][_axis];
		float neu = legs_axis_servo_neutrals_rad_[_leg][_axis];
		float pos_rad_i_n = (_pos_rad - neu) * inv;
		return(pos_rad_i_n);
	}

// ------------------------------------
    int posRadDutycycle(float _pos_rad)
	{
        int usec = PWM_USEC_NEUTRAL + (PWM_USEC_PER_RAD * _pos_rad);
        int usec_limited = std::max(PWM_USEC_MIN, std::min(usec, PWM_USEC_MAX));
        return usec_limited;
	}

// ------------------------------------
    float deg2rad(float deg)
    {
        return deg * PI / 180.0f;
    }

// ------------------------------------
    float rad2deg(float rad)
    {
        return rad * 180.0f / PI;
    }

// 非平行リンク機構の導入 -------------
// for PrintPupper v0.2's unparallel link mechanism
	void extrapolationUnparallelActuatorPositions()
	{
		for(int leg_index = 0; leg_index < LEGS; leg_index++){
			float L0Rad = legs_axis_msg_pos_[leg_index][HIP];
			float L1Rad = legs_axis_msg_pos_[leg_index][LEG];
			float L2Rad = legs_axis_msg_pos_[leg_index][KNE];
			float L3Rad = 0;
			float L3RadSafe = 0;
			if(culk_.compute(L1Rad, L2Rad, &L3Rad)){
				L3RadSafe = L3Rad;
			}else{
				;
			}
			legs_axis_cmd_pos_[leg_index][HIP] = L0Rad;
			legs_axis_cmd_pos_[leg_index][LEG] = L1Rad;
			legs_axis_cmd_pos_[leg_index][KNE] = L3RadSafe;
		}
		return;
	}
};

// ----------------------------------------------------------------------------
int main(int argc, char** argv)
{
	// Entry point of ROS node
	ros::init(argc, argv, "pp_champ_joints_hardware");
	PpChampJointsHardware *pcjh = new PpChampJointsHardware();
	ros::spin();
	delete pcjh;
	return 0;
}

// ----------------------------------------------------------------------------
std::vector<std::vector<float>> loadFloatArray(const std::string& filename)
{
    std::ifstream f(filename);
    std::string s((std::istreambuf_iterator<char>(f)), {});
    std::vector<std::vector<float>> a;
    std::vector<float> row;
    const char* p = s.c_str();

    while (*p) {
        if (*p == '[') {
            ++p;

            // "[[" の外側は無視
            if (*p == '[')
                continue;

            row.clear();

            while (*p && *p != ']') {
                while (*p == ' ' || *p == ',')
                    ++p;

                char* end;
                float v = std::strtof(p, &end);

                if (end != p) {
                    row.push_back(v);
                    p = end;
                } else {
                    ++p;
                }
            }

            if (!row.empty())
                a.push_back(row);
        } else {
            ++p;
        }
    }
    return a;
}

