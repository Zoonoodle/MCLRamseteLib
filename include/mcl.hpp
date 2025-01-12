#pragma once

#include <vector>
#include <random>      // for std::default_random_engine, distributions
#include "pros/imu.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>
#include "particle.hpp "




/**
 * MCL (Monte Carlo Localization) class :O
 */
class MCL {
public:
  /**
   * Constructor
   * @param numParticles  Number of particles in the filter
   * @param x_init        Initial x guess (in cm)
   * @param y_init        Initial y guess (in cm)
   * @param theta_init    Initial orientation (radians)
   * @param fieldWidth    Field width (cm)
   * @param fieldHeight   Field height (cm)
   * @param dedicatedImuPtr (optional) pointer to a dedicated pros::Imu for MCL
   */
  MCL(int numParticles,
      double x_init, double y_init, double theta_init,
      double fieldWidth, double fieldHeight,
      pros::Imu* dedicatedImuPtr = nullptr);

  // Predict robot motion given deltaTrans (cm) and deltaRot (rad)
  void predict(double deltaTrans, double deltaRot);

  // Overloaded predict: if you want MCL to read the IMU for orientation
  void predictFromIMU(double deltaTrans);

  // Update sensor weights with 4 distance readings (cm)
  void updateWeights(double frontDist, double rightDist,
                     double backDist, double leftDist);

  // Resample particles
  void resample();

  // Get best or mean particle
  Particle getBestParticle() const;
  Particle getMeanParticle() const;

private:
  // Expected distance to walls from a given particle
  double expectedFrontDistance(const Particle &p) const;
  double expectedRightDistance(const Particle &p) const;
  double expectedBackDistance(const Particle &p) const;
  double expectedLeftDistance(const Particle &p) const;

private:
  int numParticles_;

  // Particle set
  std::vector<Particle> particles_;

  // RNG for adding motion & sensor noise
  std::default_random_engine generator_;

  // Noise parameters
  double transNoiseStd_;   // for translation
  double rotNoiseStd_;     // for rotation
  double sensorNoiseStd_;  // for distance sensors

  // Field dimensions
  double fieldWidth_;
  double fieldHeight_;

  // Optional dedicated IMU for MCL
  pros::Imu* dedicatedIMU_;

  // Track last IMU heading (radians) if using predictFromIMU
  double lastIMUHeadingRad_ = 0.0;
};

