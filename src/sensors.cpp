#include "sensors.hpp"

// Global IMU objects
pros::Imu imu(21);
pros::Imu mclImu(17);

// Global Distance sensors
pros::Distance distFront(8);
pros::Distance distRight(9);
pros::Distance distBack(3);
pros::Distance distLeft(2);

void sensorsInit() {
  // IMUs can take ~2s to calibrate
  imu.reset();
  mclImu.reset();

  while (imu.is_calibrating() || mclImu.is_calibrating()) {
    pros::delay(20);
  }
  // Dist sensors usually don't need special calibration
}

double getRobotHeadingDeg() {
  // Weighted average or simple average of headings
  double h1 = imu.get_heading(); // returns [0..359)
  double h2 = mclImu.get_heading();
  return (h1 + h2) / 2.0; 
}

// double getFrontDistance() {
//   return distFront.get();  // returns mm by default
// }

// double getRightDistance() {
//   return distRight.get();
// }

// double getBackDistance() {
//   return distBack.get();
// }

// double getLeftDistance() {
//   return distLeft.get();
// }
