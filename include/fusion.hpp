#pragma once

#include "api.h"
#include "pros/apix.h"
#include "pros/rtos.hpp"

// Forward-declare your MCL class and LemLib's chassis:
#include "mcl.hpp"                 // or wherever your MCL is declared
#include "lemlib/api.hpp"

// Forward-declare distance sensors (extern) if they're in another file:
extern pros::Distance distFront;
extern pros::Distance distRight;
extern pros::Distance distBack;
extern pros::Distance distLeft;

/**
 * A class that fuses LemLib's odometry with MCL's updates.
 *
 * Workflow:
 *  1) We read the incremental change in LemLib's pose (inches, degrees).
 *  2) Convert to cm, radians -> feed into mcl->predict(...).
 *  3) Read 4 distance sensors -> mcl->updateWeights(...), resample().
 *  4) Get MCL's best or mean particle -> convert back to inches/degrees.
 *  5) "Snap" LemLib's pose to that fused pose -> chassis.setPose(...).
 *
 * This happens in a background task (50ms loop by default).
 */
class Fusion {
public:
  /**
   * Constructor
   * @param chassis - reference to your LemLib chassis
   * @param mclObj  - pointer to your MCL instance
   */
   Fusion(lemlib::Chassis& chassis, MCL* mclObj, double blendFactor = 0.2);

  /**
   * Starts the background task that continuously fuses MCL + LemLib odometry.
   */
  void startFusionTask(int loopDelayMs = 50);

private:
  // The actual task function that runs in a loop
  static void fusionTaskThunk(void* param);
  void fusionTask(int loopDelayMs);

  // References to your chassis and MCL
  lemlib::Chassis& chassis_;
  MCL* mcl_;

  // We keep track of the previous chassis pose
  double oldX_in_;       // inches
  double oldY_in_;       // inches
  double oldTheta_deg_;  // degrees
  double blendFactor_;
};
