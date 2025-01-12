#pragma once

#include "pros/apix.h"
#include "lemlib/chassis/chassis.hpp"
#include "mcl.hpp" // or wherever your MCL is
#include "pros/rtos.hpp"
#include "sensors.hpp"
class Fusion {
public:
  // Constructor
  Fusion(lemlib::Chassis& chassis, MCL* mclObj, double blendFactor = 0.2);

  // This is the function we call in initialize() to start the background task
  void startFusionTask(int loopDelayMs = 50);

private:
  // The static "thunk" that tasks call
  static void fusionTaskThunk(void* param);

  // The main loop that does MCL + LemLib fusion
  void fusionTask(int loopDelayMs);

  // References
  lemlib::Chassis& chassis_;
  MCL* mcl_;

  // Used for incremental odometry
  double oldX_in_;
  double oldY_in_;
  double oldTheta_deg_;

  // Blend factor for partial corrections
  double blendFactor_;
};
