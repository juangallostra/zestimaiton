/* 
   zestimation.cpp: Definitons of the estimator class' methods

   Copyright (c) 2018 Juan Gallostra

   This file is part of the Arduino Range-Baro-AltitudeEstimation library.

   The Arduino Range-Baro-AltitudeEstimation library is free software:
   you can redistribute it and/or modify it under the terms of the GNU
   General Public License as published by the Free Software Foundation,
   either version 3 of the License, or (at your option) any later version.

   The Arduino Range-Baro-AltitudeEstimation library is distributed in the
   hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
   implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
   See the GNU General Public License for more details.
   <http://www.gnu.org/licenses/>.
 */

#include "zestimation.h"
#include <cmath>

namespace zestimation {

  AltitudeEstimator::AltitudeEstimator(float gain, float sigmaAccel, float sigmaGyro,
                                       float sigmaBaro, float ca, float accelThreshold):
  twoStep(sigmaAccel, sigmaGyro, sigmaBaro, ca, accelThreshold)
  {
      _gain = gain;
  }

  void AltitudeEstimator::init(void)
  {
      _estimatedAltitude = 0;
      range.init();
  }
  
  void AltitudeEstimator::estimate(float accel[3], float gyro[3], float rangeData, float baroData, uint32_t timestamp)
  {
      // first update each sensor's estimation
      // XXX timestamp units?
      twoStep.estimate(accel, gyro, baroData, timestamp);
      range.update(accel, gyro, rangeData);
      // obtain the altitude deltas
      float twoStepDelta = twoStep.getDeltaAltitude();
      float rangeDelta = range.getDeltaAltitude();
      // Combine them with a complementary filter
      if (range.isAtGround)
      {
        _estimatedAltitude = 0;
        return;
      }
      float alpha = exp( - fabs(twoStepDelta - rangeDelta) * _gain);
      _estimatedAltitude += alpha * rangeDelta;
      _estimatedAltitude += (1 - alpha) * twoStepDelta; 
  }

  float AltitudeEstimator::getAltitude(void)
  {
      return _estimatedAltitude;
  }

} // namespace cp
