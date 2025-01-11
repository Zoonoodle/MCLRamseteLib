#include "mcl.hpp"
#include <cmath>         
#include <algorithm>     
#include <numeric>       
#include <iostream>      

MCL::MCL(int numParticles, double x_init, double y_init, double theta_init,

    double fieldWidth, double fieldHeight)
  : numParticles_(numParticles),
    transNoiseStd_(1.0),     
    rotNoiseStd_(0.05),      
    sensorNoiseStd_(3.0),    
    fieldWidth_(fieldWidth),
    fieldHeight_(fieldHeight)


{
  
  particles_.reserve(numParticles_);


  for (int i = 0; i < numParticles_; i++) {
    Particle p(x_init, y_init, theta_init, 1.0 / numParticles_);
    particles_.push_back(p);
  }
}

void MCL::predict(double deltaTrans, double deltaRot) {
  
  std::normal_distribution<double> distTrans(0.0, transNoiseStd_);
  std::normal_distribution<double> distRot(0.0, rotNoiseStd_);

  for (auto &p : particles_) {
    double noisyTrans = deltaTrans + distTrans(generator_);
    double noisyRot   = deltaRot   + distRot(generator_);

    
    p.x     += noisyTrans * std::cos(p.theta);
    p.y     += noisyTrans * std::sin(p.theta);
    p.theta += noisyRot;

    
    while (p.theta > M_PI)  p.theta -= 2.0 * M_PI;
    while (p.theta < -M_PI) p.theta += 2.0 * M_PI;
  }
}

void MCL::updateWeights(double frontDist, double rightDist, 
                        double backDist, double leftDist) 
{
  double sumWeights = 0.0;

  for (auto &p : particles_) {
    
    double eFront = expectedFrontDistance(p);
    double eRight = expectedRightDistance(p);
    double eBack  = expectedBackDistance(p);
    double eLeft  = expectedLeftDistance(p);

    
    double df = frontDist - eFront;
    double dr = rightDist - eRight;
    double db = backDist  - eBack;
    double dl = leftDist  - eLeft;

    
    
    auto gauss = [&](double diff){
      return std::exp(-0.5 * (diff * diff) / (sensorNoiseStd_ * sensorNoiseStd_));
    };

    double wf = gauss(df);
    double wr = gauss(dr);
    double wb = gauss(db);
    double wl = gauss(dl);

    double combinedWeight = wf * wr * wb * wl;
    p.weight = combinedWeight;
    sumWeights += combinedWeight;
  }

  
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

void MCL::resample() {
  
  std::vector<Particle> newParticles;
  newParticles.reserve(numParticles_);

  
  std::vector<double> cdf(numParticles_);
  cdf[0] = particles_[0].weight;
  for (int i = 1; i < numParticles_; i++) {
    cdf[i] = cdf[i - 1] + particles_[i].weight;
  }

  std::uniform_real_distribution<double> distU(0.0, 1.0);

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

  Particle mean(xSum, ySum, avgTheta, 1.0);
  return mean;
}




double MCL::expectedFrontDistance(const Particle &p) const {
  
  double dist = fieldHeight_ - p.y;
  if (dist < 0) dist = 0;
  return dist;
}

double MCL::expectedRightDistance(const Particle &p) const {
  
  double dist = fieldWidth_ - p.x;
  if (dist < 0) dist = 0;
  return dist;
}

double MCL::expectedBackDistance(const Particle &p) const {
  
  double dist = p.y;
  if (dist < 0) dist = 0;
  return dist;
}

double MCL::expectedLeftDistance(const Particle &p) const {
  
  double dist = p.x;
  if (dist < 0) dist = 0;
  return dist;
}
