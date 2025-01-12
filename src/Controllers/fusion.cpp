#include "fusion.hpp"
#include <cmath> // for conversions

// fusion.cpp (excerpt)
Fusion::Fusion(lemlib::Chassis& chassis, MCL* mclObj, double blendFactor)
  : chassis_(chassis),
    mcl_(mclObj),
    oldX_in_(0.0),
    oldY_in_(0.0),
    oldTheta_deg_(0.0),
    blendFactor_(blendFactor) // store the blend factor
{
  auto initialPose = chassis_.getPose();
  oldX_in_      = initialPose.x;
  oldY_in_      = initialPose.y;
  oldTheta_deg_ = initialPose.theta;
}


void Fusion::fusionTask(int loopDelayMs) {
  while (true) {
    // 1) LemLib incremental odometry
    lemlib::Pose newPose = chassis_.getPose();
    double dx_in = newPose.x      - oldX_in_;
    double dy_in = newPose.y      - oldY_in_;
    double dth_deg = newPose.theta - oldTheta_deg_;

    // 2) Convert to cm/rad for MCL
    double dx_cm = dx_in * 2.54;
    double dy_cm = dy_in * 2.54;
    double dth_rad = dth_deg * M_PI / 180.0;
    double distForward_cm = std::sqrt(dx_cm * dx_cm + dy_cm * dy_cm);

    // 3) MCL predict + sensor update
    mcl_->predict(distForward_cm, dth_rad);
    double f = distFront.get() / 10.0; 
    double r = distRight.get() / 10.0;
    double b = distBack.get()  / 10.0;
    double l = distLeft.get()  / 10.0;
    mcl_->updateWeights(f, r, b, l);
    mcl_->resample();

    // 4) Convert MCL best to in/deg
    Particle bestP = mcl_->getBestParticle();
    float fusedX_in  = bestP.x     * 0.393701f;
    float fusedY_in  = bestP.y     * 0.393701f;
    float fusedTh_deg= bestP.theta * (180.0f / M_PI);

    // 5) **BLEND** the pose
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

    // 6) Update old odom
    oldX_in_      = newPose.x;
    oldY_in_      = newPose.y;
    oldTheta_deg_ = newPose.theta;

    pros::delay(loopDelayMs);
  }
}
