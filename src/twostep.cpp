/*
    twostep.cpp: Altitude estimation via barometer/accelerometer fusion
*/

#include "filters.h"
#include "algebra.h"
#include "twostep.h"

namespace zestimation {

  TwoStepEstimator::TwoStepEstimator(float sigmaAccel, float sigmaGyro, float sigmaBaro,
                                     float ca, float accelThreshold)
  :kalman(ca, sigmaGyro, sigmaAccel), complementary(sigmaAccel, sigmaBaro, accelThreshold)
  {
        this->sigmaAccel = sigmaAccel;
        this->sigmaGyro = sigmaGyro;
        this->sigmaBaro = sigmaBaro;
        this->ca = ca;
        this->accelThreshold = accelThreshold;
        baro.init();
  }

  void TwoStepEstimator::estimate(float accel[3], float gyro[3], float pressure, uint32_t timestamp)
  {
          baro.update(pressure);
          float baroHeight = baro.getAltitude();
          float deltat = (float)(timestamp-previousTime)/1000000.0f;
          float verticalAccel = kalman.estimate(pastGyro,
                                                pastAccel,
                                                deltat);
          complementary.estimate(& estimatedVelocity,
                                 & estimatedAltitude,
                                 baroHeight,
                                 pastAltitude,
                                 pastVerticalVelocity,
                                 pastVerticalAccel,
                                 deltat);
          // update values for next iteration
          copyVector(pastGyro, gyro);
          copyVector(pastAccel, accel);
          // Update delta altitude
          _deltaAltitude = estimatedAltitude - pastAltitude;
          pastAltitude = estimatedAltitude;
          pastVerticalVelocity = estimatedVelocity;
          pastVerticalAccel = verticalAccel;
          previousTime = timestamp;
  }

  float TwoStepEstimator::getDeltaAltitude()
  {
          // return the last estimated altitude
          return _deltaAltitude;
  }

  float TwoStepEstimator::getAltitude()
  {
          // return the last estimated altitude
          return estimatedAltitude;
  }

  float TwoStepEstimator::getVerticalVelocity()
  {
          // return the last estimated vertical velocity
          return estimatedVelocity;
  }

  float TwoStepEstimator::getVerticalAcceleration()
  {
          // return the last estimated vertical acceleration
          return pastVerticalAccel;
  }

} // namespace zestimation