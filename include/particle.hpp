#pragma once

#include "main.h"

struct Particle {
  double x;
  double y;
  double theta;   // orientation, in radians
  double weight;  // importance weight

  Particle()
      : x(0), y(0), theta(0), weight(1.0) {}

  Particle(double x_, double y_, double theta_, double w_ = 1.0)
      : x(x_), y(y_), theta(theta_), weight(w_) {}
};
