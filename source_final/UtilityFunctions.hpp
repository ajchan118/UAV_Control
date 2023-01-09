#pragma once

#include "MainLoopTypes.hpp"

//! Maps from a desired speed [rad/s] to a command output value
//  which is dimensionless
int pwmCommandFromSpeed(float desiredSpeed_rad_per_sec);

//! Maps from a desired force [N] to a command speed [rad/s]
float speedFromForce(float desiredForce_N);
