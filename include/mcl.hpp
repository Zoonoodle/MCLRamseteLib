#pragma once

#include <vector>
#include <random>
#include "particle.hpp"

/**
 * Monte Carlo Localization (MCL) class.
 * 
 * Uses a set of particles to estimate (x, y, theta).
 * Sensor model uses four distance sensors to measure distance 
 * to each wall of a 2D field.
 */
class MCL {
public:
  /**
   * Constructor
   * @param numParticles  number of particles in the filter
   * @param x_init        initial x guess
   * @param y_init        initial y guess
   * @param theta_init    initial orientation (radians)
   * @param fieldWidth    width of the field in cm
   * @param fieldHeight   height of the field in cm
   */
  MCL(int numParticles, double x_init, double y_init, double theta_init,
      double fieldWidth = 365.76, double fieldHeight = 365.76);

  /**
   * Predict step (motion model).
   * @param deltaTrans  forward distance traveled (cm)
   * @param deltaRot    change in orientation (radians)
   */
  void predict(double deltaTrans, double deltaRot);

  /**
   * Update step (sensor model) using four distance sensor readings
   * in cm. (front, right, back, left).
   */
  void updateWeights(double frontDist, double rightDist, 
                     double backDist, double leftDist);

  /**
   * Resample step. 
   * Uses a simple roulette wheel (or low-variance) sampling to 
   * discard improbable particles and replicate probable ones.
   */
  void resample();

  /** 
   * Returns the particle with the highest weight.
   */
  Particle getBestParticle() const;

  /** 
   * Returns a weighted mean of all particles (x, y, orientation). 
   */
  Particle getMeanParticle() const;

private:
  int numParticles_;
  std::vector<Particle> particles_;
  std::default_random_engine generator_;

  // Noise parameters for motion model & sensor
  double transNoiseStd_;   // e.g. ~1 cm
  double rotNoiseStd_;     // e.g. ~0.05 rad (~3 deg)
  double sensorNoiseStd_;  // e.g. ~3 cm

  // Field dimensions in cm
  double fieldWidth_;
  double fieldHeight_;

  // Private helpers for expected distances from each side
  double expectedFrontDistance(const Particle &p) const;
  double expectedRightDistance(const Particle &p) const;
  double expectedBackDistance(const Particle &p) const;
  double expectedLeftDistance(const Particle &p) const;
};
