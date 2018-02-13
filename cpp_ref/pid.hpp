//
// Written by Andrey Leshenko and Eli Tarnarutsky, 2017.
// Published under the MIT license.
//

#pragma once

class Pid
{
public:
	float kp;
	float ki;
	float kd;
	float lastValue = 0;
	float scaledErrorSum = 0;
	float minIntegral = -1;
	float maxIntegral = 1;

	Pid() {};

	Pid(float kp, float ki, float kd)
		: kp{kp},
		ki{ki},
		kd{kd}
	{ };

	Pid(float kp, float ki, float kd, float minI, float maxI)
		: kp{kp},
		ki{ki},
		kd{kd},
		minIntegral{minI},
		maxIntegral{maxI}
	{ };

	float calculate(float currentValue, float setPoint, float deltaTime);
	float calculate(float error, float deltaTime);
};
