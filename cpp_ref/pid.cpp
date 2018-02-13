//
// Written by Andrey Leshenko and Eli Tarnarutsky, 2017.
// Published under the MIT license.
//

#include "pid.hpp"

static float clamp(float value, float min, float max)
{
	if (value > max) return max;
	if (value < min) return min;

	return value;
}

static float clamp1(float value)
{
	return clamp(value, -1, 1);
}

float Pid::calculate(float currentValue, float setPoint, float deltaTime)
{
	float error = setPoint - currentValue;

	scaledErrorSum += ki * (error * deltaTime);
	scaledErrorSum = clamp(scaledErrorSum, minIntegral, maxIntegral);
	float errorDerivative = -(currentValue - lastValue) / deltaTime;
	lastValue = currentValue;

	return clamp1(kp * error + scaledErrorSum + kd * errorDerivative);
}

float Pid::calculate(float error, float deltaTime)
{
	return calculate(-error, 0, deltaTime);
}
