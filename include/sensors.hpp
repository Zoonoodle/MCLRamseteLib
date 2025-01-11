#pragma once

#include "api.h"

/**
 * Extern declarations for sensor objects.
 * We'll define them in sensors.cpp
 */
extern pros::Imu imu1;
extern pros::Imu imu2;

extern pros::Distance distFront;
extern pros::Distance distRight;
extern pros::Distance distBack;
extern pros::Distance distLeft;

/**
 * Initialize all sensors. 
 * Resets/calibrates IMUs and ensures they're ready.
 */
void sensorsInit();

/**
 * Returns a fused robot heading in degrees [0..360) from two IMUs.
 * Or you could do get_rotation() if you prefer -∞..∞.
 */
double getRobotHeadingDeg();

/**
 * Get raw distance sensor readings in mm (PROS default)
 * or convert them to cm as needed.
 */
double getFrontDistance();
double getRightDistance();
double getBackDistance();
double getLeftDistance();
