#include "main.h"
#include "mcl.hpp"
#include "sensors.hpp"
#include "motion_controller.hpp"
#include <iostream>

// Globally or statically define your objects
pros::MotorGroup left_motors({-16, -18, -6}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({19, 2, 12}, pros::MotorGearset::blue);

MCL *mcl = nullptr;

// For velocity PID on each side
VelocityPID leftVelPID(1.0, 0.0, 0.0, 1000.0);
VelocityPID rightVelPID(1.0, 0.0, 0.0, 1000.0);



void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */void initialize() {
  sensorsInit();
  // Example: 100 particles
  mcl = new MCL(100, 50, 50, 0.0);
  // Tare motors for odom
  left_motors.tare_position();
  right_motors.tare_position();
}

void autonomous() {
  // 1) MCL step (like before) or do it in a separate loop.
  // Just a small example: we do a quick update cycle:
  for (int i = 0; i < 5; i++) {
    auto [deltaTrans, deltaRot] = getOdomDelta();
    mcl->predict(deltaTrans, deltaRot);
    double f = getFrontDistance()/10.0;
    double r = getRightDistance()/10.0;
    double b = getBackDistance()/10.0;
    double l = getLeftDistance()/10.0;
    mcl->updateWeights(f, r, b, l);
    mcl->resample();
    pros::delay(50);
  }
  
  // 2) Suppose we want to drive forward 200 cm using a trapezoidal profile
  TrapezoidalProfile profile(200.0, 100.0, 50.0);
  // e.g., maxVel = 100 cm/s, accel = 50 cm/s^2
  // total time
  double totalT = profile.getTotalTime();

  // 3) Control loop: follow the velocity from the profile
  // We'll sample the profile in small time increments (say 10 ms)
  double startTime = pros::millis();
  double dt = 10; // ms
  while (true) {
    double t = (pros::millis() - startTime) / 1000.0; // seconds
    if (t > totalT) break;

    // desired velocity in cm/s
    double desiredVelCS = profile.getVelocity(t);

    // Convert to motor RPM
    // 4" wheel ~ 31.9 cm circumference
    double wheelCirc = 31.9; 
    double revPerSec = desiredVelCS / wheelCirc;
    double rpm = revPerSec * 60.0;

    // Set the velocity PID's target
    leftVelPID.setTarget(rpm);
    rightVelPID.setTarget(rpm);

    // Get actual velocity from motor group in RPM
    double leftRPM = left_motors.getActualVelocity();  // PROS motorGroup typically returns rpm
    double rightRPM = right_motors.getActualVelocity();

    // Calculate output
    double leftOutput = leftVelPID.update(leftRPM);
    double rightOutput = rightVelPID.update(rightRPM);

    // Send voltages to motors
    left_motors.moveVoltage(leftOutput);
    right_motors.moveVoltage(rightOutput);

    pros::delay(dt); 
  }

  // 4) Stop the motors at the end
  left_motors.moveVoltage(0);
  right_motors.moveVoltage(0);

  // (Optionally) do more MCL updates to refine localization
  // ...
}
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}