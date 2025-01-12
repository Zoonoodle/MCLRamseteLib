#include "fusion.hpp"
#include <cmath> // for conversions

Fusion::Fusion(lemlib::Chassis& chassis, MCL* mclObj)
  : chassis_(chassis),
    mcl_(mclObj),
    oldX_in_(0.0),
    oldY_in_(0.0),
    oldTheta_deg_(0.0)
{
  // On construction, we can store the current chassis pose
  auto initialPose = chassis_.getPose();
  oldX_in_      = initialPose.x;
  oldY_in_      = initialPose.y;
  oldTheta_deg_ = initialPose.theta; // typically 0 if just started
}

void Fusion::startFusionTask(int loopDelayMs) {
  // Refresh the initial pose
  auto initialPose = chassis_.getPose();
  oldX_in_      = initialPose.x;
  oldY_in_      = initialPose.y;
  oldTheta_deg_ = initialPose.theta;

  // Launch a PROS task for continuous fusion
  pros::Task fusionTask(fusionTaskThunk, this, "MCL-Fusion-Task");
}

// Static thunk: calls the member fusionTask
void Fusion::fusionTaskThunk(void* param) {
  // param is 'this'
  auto* self = static_cast<Fusion*>(param);
  // We can choose a default loop delay or pass it
  self->fusionTask(50);
}

// The main loop
void Fusion::fusionTask(int loopDelayMs) {
  while (true) {
    // 1) Get current LemLib pose in inches/degrees
    lemlib::Pose newPose = chassis_.getPose();
    double dx_in = newPose.x      - oldX_in_;
    double dy_in = newPose.y      - oldY_in_;
    double dth_deg = newPose.theta - oldTheta_deg_;

    // 2) Convert to cm and radians for MCL
    double dx_cm = dx_in * 2.54;
    double dy_cm = dy_in * 2.54;
    double dth_rad = dth_deg * M_PI / 180.0;

    // For MCL, we might approximate "forward distance" as the Euclidean distance
    double distForward_cm = std::sqrt(dx_cm*dx_cm + dy_cm*dy_cm);

    // *** Predict step ***
    // Option A: pass dist + turn
    mcl_->predict(distForward_cm, dth_rad);

    // Option B (if your MCL wants to read a dedicated IMU):
    // double distForward_cm = ...
    // mcl_->predictFromIMU(distForward_cm);

    // 3) Read four distance sensors (assuming .get() returns mm)
    double f = distFront.get() / 10.0;  // cm
    double r = distRight.get() / 10.0;
    double b = distBack.get()  / 10.0;
    double l = distLeft.get()  / 10.0;

    // *** Update / Resample ***
    mcl_->updateWeights(f, r, b, l);
    mcl_->resample();

    // 4) Get MCL's best (or mean) pose and convert back to inches/degrees
    Particle bestP = mcl_->getBestParticle();
    float fusedX_in  = bestP.x     * 0.393701;   // cm -> in
    float fusedY_in  = bestP.y     * 0.393701;
    float fusedTh_deg= bestP.theta * (180.0 / M_PI);

    // 5) Snap LemLib's odometry to the fused pose
    chassis_.setPose({fusedX_in, fusedY_in, fusedTh_deg});

    // 6) Update old pose for next iteration
    oldX_in_      = newPose.x;
    oldY_in_      = newPose.y;
    oldTheta_deg_ = newPose.theta;

    // Delay
    pros::delay(loopDelayMs);
  }
}
