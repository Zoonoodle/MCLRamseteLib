#include "fusion.hpp"
#include <cmath> // for conversions

// 1) Constructor
Fusion::Fusion(lemlib::Chassis& chassis, MCL* mclObj, double blendFactor)
  : chassis_(chassis),
    mcl_(mclObj),
    oldX_in_(0.0),
    oldY_in_(0.0),
    oldTheta_deg_(0.0),
    blendFactor_(blendFactor)
{
  // Store the initial pose from LemLib
  auto initialPose = chassis_.getPose();
  oldX_in_      = initialPose.x;
  oldY_in_      = initialPose.y;
  oldTheta_deg_ = initialPose.theta;
}

// 2) The function that spawns the background task
void Fusion::startFusionTask(int loopDelayMs) {
  // optionally refresh old pose here, if desired
  auto initialPose = chassis_.getPose();
  oldX_in_      = initialPose.x;
  oldY_in_      = initialPose.y;
  oldTheta_deg_ = initialPose.theta;

  // Create a PROS task that calls fusionTaskThunk
  pros::Task fusionTask(fusionTaskThunk, this, "FusionTask");
}

// 3) The static thunk that the PROS task calls
void Fusion::fusionTaskThunk(void* param) {
  auto* self = static_cast<Fusion*>(param);
  // We call the member function with default or chosen loopDelayMs
  self->fusionTask(50); 
}

// 4) The main fusion loop
void Fusion::fusionTask(int loopDelayMs) {
  while(true) {
    // 1) Get current LemLib pose
    lemlib::Pose newPose = chassis_.getPose();
    double dx_in = newPose.x - oldX_in_;
    double dy_in = newPose.y - oldY_in_;
    double dth_deg = newPose.theta - oldTheta_deg_;

    // 2) Convert for MCL
    double dx_cm   = dx_in * 2.54;
    double dy_cm   = dy_in * 2.54;
    double dth_rad = dth_deg * M_PI / 180.0;
    double distForward_cm = std::sqrt(dx_cm*dx_cm + dy_cm*dy_cm);

    // 3) MCL predict + sensor update
    mcl_->predict(distForward_cm, dth_rad);
    double f = distFront.get() / 10.0; // mm->cm
    double r = distRight.get() / 10.0;
    double b = distBack.get()  / 10.0;
    double l = distLeft.get()  / 10.0;
    mcl_->updateWeights(f, r, b, l);
    mcl_->resample();

    // 4) Convert MCL pose to in/deg
    Particle bestP = mcl_->getBestParticle();
    float fusedX_in  = bestP.x     * 0.393701f;
    float fusedY_in  = bestP.y     * 0.393701f;
    float fusedTh_deg= bestP.theta * (180.0f / M_PI);

    // 5) Blend or Snap
    lemlib::Pose odomPose = chassis_.getPose(); // or newPose
    float alpha = static_cast<float>(blendFactor_);

    float diffX  = fusedX_in  - odomPose.x;
    float diffY  = fusedY_in  - odomPose.y;
    float diffTh = fusedTh_deg - odomPose.theta;

    // partial correction
    float blendedX  = odomPose.x + alpha * diffX;
    float blendedY  = odomPose.y + alpha * diffY;
    float blendedTh = odomPose.theta + alpha * diffTh;

    chassis_.setPose({blendedX, blendedY, blendedTh});

    // 6) Save for next iteration
    oldX_in_      = newPose.x;
    oldY_in_      = newPose.y;
    oldTheta_deg_ = newPose.theta;

    pros::delay(loopDelayMs);
  }
}
