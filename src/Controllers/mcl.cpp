#include "mcl.hpp"
#include <cmath>         // std::sin, std::cos, M_PI
#include <algorithm>     // std::max_element, std::lower_bound


// ------------------------------
// 1) Constructor
// ------------------------------
MCL::MCL(int numParticles,
         double x_init, double y_init, double theta_init,
         double fieldWidth, double fieldHeight,
         pros::Imu* dedicatedImuPtr)
  : numParticles_(numParticles),
    transNoiseStd_(1.0),
    rotNoiseStd_(0.05),
    sensorNoiseStd_(3.0),
    fieldWidth_(fieldWidth),
    fieldHeight_(fieldHeight),
    dedicatedIMU_(dedicatedImuPtr)  // store pointer to the "MCL IMU"
{
  particles_.reserve(numParticles_);
  for (int i = 0; i < numParticles_; i++) {
    Particle p(x_init, y_init, theta_init, 1.0 / numParticles_);
    particles_.push_back(p);
  }
}

// ------------------------------
// 2) PREDICT Step
// ------------------------------
void MCL::predict(double deltaTrans, double deltaRot) {
  // Add random noise
  std::normal_distribution<double> distTrans(0.0, transNoiseStd_);
  std::normal_distribution<double> distRot(0.0, rotNoiseStd_);

  for (auto &p : particles_) {
    double noisyTrans = deltaTrans + distTrans(generator_);
    double noisyRot   = deltaRot   + distRot(generator_);

    // Move particle forward
    p.x     += noisyTrans * std::cos(p.theta);
    p.y     += noisyTrans * std::sin(p.theta);
    p.theta += noisyRot;

    // Keep theta in [-pi, pi]
    while (p.theta >  M_PI) p.theta -= 2.0 * M_PI;
    while (p.theta < -M_PI) p.theta += 2.0 * M_PI;
  }
}

// Overload: If you want MCL to read from the dedicated IMU directly
// and handle the "deltaRot" itself, you can do something like this:
void MCL::predictFromIMU(double deltaTrans) {
  if (!dedicatedIMU_) return; // safety check

  // Grab the IMU's heading. E.g. get_heading() returns [0..359)
  double currentHeadingDeg = dedicatedIMU_->get_heading();

  // Convert to radians in [-pi..pi] or [0..2pi], up to you:
  double currentHeadingRad = currentHeadingDeg * M_PI / 180.0;

  // Compute how much the IMU heading changed since last cycle
  double dTheta = currentHeadingRad - lastIMUHeadingRad_;
  // normalize dTheta to [-pi, pi]
  while (dTheta >  M_PI) dTheta -= 2.0 * M_PI;
  while (dTheta < -M_PI) dTheta += 2.0 * M_PI;

  // Save for next iteration
  lastIMUHeadingRad_ = currentHeadingRad;

  // Now call the normal predict with that rotation
  predict(deltaTrans, dTheta);
}

// ------------------------------
// 3) UPDATE Weights Step
// ------------------------------
void MCL::updateWeights(double frontDist, double rightDist,
                        double backDist, double leftDist)
{
  double sumWeights = 0.0;

  for (auto &p : particles_) {
    // 1) Calculate expected distances from this particle's pose
    double eFront = expectedFrontDistance(p);
    double eRight = expectedRightDistance(p);
    double eBack  = expectedBackDistance(p);
    double eLeft  = expectedLeftDistance(p);

    // 2) Differences
    double df = frontDist - eFront;
    double dr = rightDist - eRight;
    double db = backDist  - eBack;
    double dl = leftDist  - eLeft;

    // 3) Gaussian likelihood for each sensor
    auto gauss = [&](double diff){
      return std::exp(-0.5 * (diff * diff) /
                      (sensorNoiseStd_ * sensorNoiseStd_));
    };

    double wf = gauss(df);
    double wr = gauss(dr);
    double wb = gauss(db);
    double wl = gauss(dl);

    // Combine
    double combinedWeight = wf * wr * wb * wl;
    p.weight = combinedWeight;
    sumWeights += combinedWeight;
  }

  // 4) Normalize
  if (sumWeights > 0) {
    for (auto &p : particles_) {
      p.weight /= sumWeights;
    }
  } else {
    double w = 1.0 / numParticles_;
    for (auto &p : particles_) {
      p.weight = w;
    }
  }
}

// ------------------------------
// 4) RESAMPLE
// ------------------------------
void MCL::resample() {
  std::vector<Particle> newParticles;
  newParticles.reserve(numParticles_);

  // Build a CDF of weights
  std::vector<double> cdf(numParticles_);
  cdf[0] = particles_[0].weight;
  for (int i = 1; i < numParticles_; i++) {
    cdf[i] = cdf[i - 1] + particles_[i].weight;
  }

  std::uniform_real_distribution<double> distU(0.0, 1.0);

  // Low-variance sampling
  for (int i = 0; i < numParticles_; i++) {
    double r = distU(generator_);
    auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
    int idx = (int)std::distance(cdf.begin(), it);
    Particle chosen = particles_[idx];
    chosen.weight = 1.0 / numParticles_;
    newParticles.push_back(chosen);
  }

  particles_ = newParticles;
}

// ------------------------------
// 5) Get Best or Mean Particle
// ------------------------------
Particle MCL::getBestParticle() const {
  auto it = std::max_element(particles_.begin(), particles_.end(),
    [](const Particle &a, const Particle &b){
      return a.weight < b.weight;
    }
  );
  return (it == particles_.end()) ? Particle() : *it;
}

Particle MCL::getMeanParticle() const {
  double xSum = 0.0, ySum = 0.0;
  double cosSum = 0.0, sinSum = 0.0;

  for (auto &p : particles_) {
    xSum    += p.x * p.weight;
    ySum    += p.y * p.weight;
    cosSum  += std::cos(p.theta) * p.weight;
    sinSum  += std::sin(p.theta) * p.weight;
  }
  double avgTheta = std::atan2(sinSum, cosSum);

  return Particle(xSum, ySum, avgTheta, 1.0);
}

// ------------------------------
// 6) Expected Distances (Walls)
// ------------------------------

// If the field origin (0,0) is bottom-left, and fieldWidth_ is X dimension,
// fieldHeight_ is Y dimension, then:
double MCL::expectedFrontDistance(const Particle &p) const {
  double dist = fieldHeight_ - p.y;
  return (dist < 0) ? 0 : dist;
}

double MCL::expectedRightDistance(const Particle &p) const {
  double dist = fieldWidth_ - p.x;
  return (dist < 0) ? 0 : dist;
}

double MCL::expectedBackDistance(const Particle &p) const {
  double dist = p.y;
  return (dist < 0) ? 0 : dist;
}

double MCL::expectedLeftDistance(const Particle &p) const {
  double dist = p.x;
  return (dist < 0) ? 0 : dist;
}





