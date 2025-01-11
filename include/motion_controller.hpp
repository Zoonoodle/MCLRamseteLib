#pragma once
#include "dtConfigs.hpp"
#include "api.h"
#include <cmath>

/**
 * Simple PID controller to maintain a target velocity (e.g. in RPM).
 * 
 * Typical usage:
 * 1) Construct with (kP, kI, kD, integralLimit).
 * 2) setTarget(rpm).
 * 3) On a timed loop (every 10 or 20ms), call update(currentRpm) -> motorVoltage.
 * 4) Send that voltage to the motor(s).
 */
class VelocityPID {
public:
  /**
   * Constructor.
   * @param kP            Proportional gain
   * @param kI            Integral gain
   * @param kD            Derivative gain
   * @param integralLimit Absolute clamp on the integral term
   */
  VelocityPID(double kP, double kI, double kD, double integralLimit);

  /**
   * Sets the desired velocity target (e.g., in RPM).
   */
  void setTarget(double targetRPM);

  /**
   * Runs one PID update cycle.
   * @param currentRPM  The actual measured velocity
   * @return            The control output (e.g., motor voltage in [-12000..12000])
   */
  double update(double currentRPM);

  /**
   * Resets the internal PID state (integral, previous error, etc.).
   */
  void reset();

private:
  double kP_;
  double kI_;
  double kD_;

  double integralLimit_;

  double target_;    // target velocity (RPM)
  double errorPrev_; // previous cycle error
  double integral_;  // accumulated error
};


/**
 * A simple trapezoidal motion profile for moving a certain distance in 1D.
 * 
 * It divides the motion into:
 * 1. Acceleration from 0 to maxVel
 * 2. Constant maxVel (cruise)
 * 3. Deceleration back to 0
 * 
 * If distance is too short to ever reach maxVel, it will accelerate then decelerate directly.
 */
class TrapezoidalProfile {
public:
  /**
   * Constructor.
   * @param distance  total distance to travel (cm, for example)
   * @param maxVel    maximum velocity (cm/s)
   * @param accel     acceleration (cm/s^2)
   */
  TrapezoidalProfile(double distance, double maxVel, double accel);

  /**
   * Gets the profile’s planned velocity (cm/s) at time t.
   * @param t  time in seconds from the start of the move
   * @return   velocity in cm/s
   */
  double getVelocity(double t);

  /**
   * Returns the total time from start to finish of the profile.
   */
  double getTotalTime();

private:
  double distance_;
  double maxVel_;
  double accel_;

  // Precomputed times for each segment (accel, cruise, decel)
  double tAccel_;  // time from 0 -> maxVel
  double tFull_;   // end of constant velocity segment
  double tDecel_;  // end of decel, i.e. total motion time
};


/**
 * A higher-level motion controller that:
 * 1) Uses a TrapezoidalProfile to generate velocity commands over time.
 * 2) Uses VelocityPID controllers to achieve those velocities on each side of a differential drive.
 * 
 * This class is set up to handle a single "drive straight" move. 
 * Extend or adapt for more complex paths or turning.
 */
class MotionController {
public:
  /**
   * Constructor.
   * @param leftMotors   A pros::MotorGroup (or custom group) for the left side
   * @param rightMotors  A pros::MotorGroup for the right side
   * @param wheelCirc    The wheel circumference in cm
   */
  MotionController(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors,
                   double wheelCirc);

  /**
   * Drive a specified distance in cm using a trapezoidal profile.
   * @param distance     total distance to travel (cm)
   * @param maxVel       max velocity in cm/s
   * @param accel        acceleration in cm/s^2
   * @param dt           time step for loop in ms (e.g. 10 or 20)
   */
  void driveDistance(double distance, double maxVel, double accel, int dt);

  /**
   * Optionally, set new PID gains if needed. 
   * This sets both left and right VelocityPIDs to the same gains.
   */
  void setVelocityPID(double kP, double kI, double kD, double iLimit);

private:
  // Motor references
  pros::MotorGroup& leftMotors_;
  pros::MotorGroup& rightMotors_;

  // Two velocity PID controllers (one for left, one for right)
  VelocityPID leftVelPID_;
  VelocityPID rightVelPID_;

  // Wheel circumference (cm). Used to convert linear velocity -> motor RPM.
  double wheelCirc_;

  // Helper to convert cm/s to motor RPM, given wheelCirc
  double linearToRPM(double linearSpeedCmPerSec);
};
