// codigo .h
#ifndef PID_H
#define PID_H
#include "Arduino.h"
class PID
{
	public:
		PID(int min_pwm, int max_pwm, float kp, float ki, float kd, float dt, float integral_max);
		float compute(float reference, float measured);
		int apply_deadzone(float out);
		void reset();
		void updateConstants(float kp, float ki, float kd);
	private:
		int minPWM_;
		int maxPWM_;
		float kp_;
		float ki_;
		float kd_;
		float dt_;
		float integralMax_;
		float integral_;
		float derivative_;
		float prev_measured_;
};
#endif