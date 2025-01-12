#include "main.h"
#include "mcl.hpp"
#include "pros/rtos.hpp"
#include "sensors.hpp"
#include "mcl.hpp"
#include "Chassis.hpp"
#include "fusion.hpp"

MCL* mcl = nullptr;
Fusion* fusion = nullptr; //fusion pointer


void initialize() {

  pros::lcd::initialize();
  chassis.calibrate();
  sensorsInit();
  // 2) Initialize MCL
  //  particles, initial guess (50,50,0 rad), field size 365.76x365.76 cm
  //pass pointer to the "mclImu"


  mcl = new MCL(200, 50.0, 50.0, 0.0, 365.76, 365.76, &mclImu);


 
  fusion = new Fusion(chassis, mcl);  // 3)  Fusion object to fuse LemLib odom + MCL
 
  fusion->startFusionTask(50); //50 ms loop


  pros::Task debugTask([] {
    while (true) {
      auto lemPose = chassis.getPose(); 
      Particle mclBest = mcl->getBestParticle();

   
      pros::lcd::print(0, "LEM ODOM: X=%.2f Y=%.2f TH=%.2f",
                       lemPose.x, lemPose.y, lemPose.theta);
      pros::lcd::print(1, "MCL BEST: X=%.1f Y=%.1f TH=%.1f rad",
                       mclBest.x, mclBest.y, mclBest.theta);

      pros::delay(100);
    }
  });
}

void autonomous() {
 
  chassis.moveToPose(10, 24, 90, 2000);

  

}


void disabled() {
  //mogo.setValue(false);

}

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