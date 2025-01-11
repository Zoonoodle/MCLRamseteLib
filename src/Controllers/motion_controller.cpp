#include "motion_controller.hpp"
#include <algorithm> // for std::clamp if needed
#include <cmath>
#include "pros/apix.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"




//measurements we need





/////////////////////////////////
// VelocityPID Implementation  //
/////////////////////////////////

VelocityPID::VelocityPID(double kP, double kI, double kD, double integralLimit)
  : kP_(kP), kI_(kI), kD_(kD),
    integralLimit_(integralLimit),
    target_(0), errorPrev_(0), integral_(0) {
}

void VelocityPID::setTarget(double targetRPM) {
  target_ = targetRPM;
}

double VelocityPID::update(double currentRPM) {
  double error = target_ - currentRPM;

  // Integrate the error
  integral_ += error;
  // Clamp the integral to prevent windup
  if (std::fabs(integral_) > integralLimit_) {
    integral_ = std::copysign(integralLimit_, integral_);
  }

  // Derivative
  double derivative = error - errorPrev_;

  // PID sum
  double output = (kP_ * error) + (kI_ * integral_) + (kD_ * derivative);

  // Remember for next time
  errorPrev_ = error;

  // Typical motor voltage range in PROS is -12000 to +12000
  if (output > 12000) output = 12000;
  if (output < -12000) output = -12000;

  return output;
}

void VelocityPID::reset() {
  integral_ = 0;
  errorPrev_ = 0;
  target_ = 0;
}


//////////////////////////////////////
// TrapezoidalProfile Implementation //
//////////////////////////////////////

TrapezoidalProfile::TrapezoidalProfile(double distance, double maxVel, double accel)
  : distance_(distance), maxVel_(maxVel), accel_(accel),
    tAccel_(0), tFull_(0), tDecel_(0)
{
  // Time to accelerate 0 -> maxVel
  tAccel_ = maxVel_ / accel_;

  // Distance covered in that accel phase
  double distAccel = 0.5 * accel_ * (tAccel_ * tAccel_);

  // symmetrical decel covers the same distance
  // total needed for accel+decel = 2 * distAccel
  if (2.0 * distAccel > distance_) {
    // We never reach maxVel
    // Solve for time to go 0->v->0 over 'distance'
    // distance = a * t^2 (because it's accelerate half the time, decelerate half)
    // tTotal = sqrt(distance / a)
    double tTotal = std::sqrt(distance_ / accel_);
    // We'll define that the entire motion is accel for tTotal/2, 
    // then decel for tTotal/2, but for simplicity we can just do:
    tAccel_ = tTotal;  // we accelerate from 0..peakVel
    tFull_  = tTotal;  // no cruise
    tDecel_ = tTotal;  // done
  } else {
    // We do get a cruise phase
    double distCruise = distance_ - 2.0 * distAccel;
    double tCruise = distCruise / maxVel_;
    tFull_  = tAccel_ + tCruise;
    tDecel_ = tFull_ + tAccel_;
  }
}

double TrapezoidalProfile::getVelocity(double t) {
  if (t <= 0) return 0;
  if (t >= tDecel_) return 0; // motion finished

  // Accel phase
  if (t < tAccel_) {
    return accel_ * t;  // v = a*t
  }
  // Cruise phase
  if (t < tFull_) {
    return maxVel_;
  }
  // Decel phase
  double tIntoDecel = t - tFull_; 
  double vDecel = maxVel_ - accel_ * tIntoDecel;
  return (vDecel > 0) ? vDecel : 0;
}

double TrapezoidalProfile::getTotalTime() {
  return tDecel_;
}


/////////////////////////////////////
// MotionController Implementation //
/////////////////////////////////////

MotionController::MotionController(pros::MotorGroup& leftMotors,
                                   pros::MotorGroup& rightMotors,
                                   double wheelCirc)
  : leftMotors_(leftMotors),
    rightMotors_(rightMotors),
    // Default PID gains: just placeholders (tune these!)
    leftVelPID_(1.0, 0.0, 0.0, 1000.0),
    rightVelPID_(1.0, 0.0, 0.0, 1000.0),
    wheelCirc_(wheelCirc)
{
}

void MotionController::driveDistance(double distance, double maxVel, double accel, int dt) {
  // 1. Create the trapezoidal profile
  TrapezoidalProfile profile(distance, maxVel, accel);
  double totalTime = profile.getTotalTime();

  // 2. Reset PIDs
  leftVelPID_.reset();
  rightVelPID_.reset();

  // 3. Reset motor positions (optional, so we can read actual velocity from 0)
  leftMotors_.tare_position();
  rightMotors_.tare_position();

  // 4. Timed loop to follow the profile
  double startTime = pros::millis();
  int timeStep = dt; // e.g. 10 or 20 ms

  while (true) {
    double currentMillis = pros::millis() - startTime;
    double tSec = currentMillis / 1000.0;
    if (tSec > totalTime) break;

    // get the desired linear velocity in cm/s from the profile
    double desiredVelCs = profile.getVelocity(tSec);

    // convert to motor velocity in RPM
    double targetRPM = linearToRPM(desiredVelCs);

    // measure actual velocity from the motor groups in RPM
    double leftRPM = leftMotors_.get_actual_velocity();  
    double rightRPM = rightMotors_.get_actual_velocity();

    // set each side’s PID target & compute output
    leftVelPID_.setTarget(targetRPM);
    rightVelPID_.setTarget(targetRPM);

    double leftVoltage = leftVelPID_.update(leftRPM);
    double rightVoltage = rightVelPID_.update(rightRPM);

    // apply voltages
    leftMotors_.move_voltage(leftVoltage);
    rightMotors_.move_voltage(rightVoltage);

    // wait for next cycle
    pros::delay(timeStep);
  }

  // 5. Stop at the end
  leftMotors_.move_voltage(0);
  rightMotors_.move_voltage(0);
}

void MotionController::setVelocityPID(double kP, double kI, double kD, double iLimit) {
  leftVelPID_ = VelocityPID(kP, kI, kD, iLimit);
  rightVelPID_ = VelocityPID(kP, kI, kD, iLimit);
}

double MotionController::linearToRPM(double linearSpeedCmPerSec) {
  // If wheelCirc_ is in cm, then #revs per second = linearSpeedCmPerSec / wheelCirc_
  // 1 revolution = 360 deg
  // But we want RPM (revs per minute), so multiply by 60.
  double revPerSec = linearSpeedCmPerSec / wheelCirc_;
  double rpm = revPerSec * 60.0;
  return rpm;
}
