/*
   filters.cpp: Filter class implementations
 */

#include <cmath>
#include <stdlib.h> // XXX eventually use fabs() instead of abs() ?

#include "filters.h"

namespace zestimation {

  // Low pass filter
  LowPassFilter::LowPassFilter(uint16_t historySize)
  {
      _historySize = historySize;
  }

  void LowPassFilter::init(void)
  {
      for (uint8_t k=0; k<_historySize; ++k) {
          _history[k] = 0;
      }
      _historyIdx = 0;
      _sum = 0;
  }

  float LowPassFilter::update(float value)
  {
      uint8_t indexplus1 = (_historyIdx + 1) % _historySize;
      _history[_historyIdx] = value;
      _sum += _history[_historyIdx];
      _sum -= _history[indexplus1];
      _historyIdx = indexplus1;
      return _sum / (_historySize - 1);
  }


  // Kalman filter
  void KalmanFilter::getPredictionCovariance(float covariance[3][3], float previousState[3], float deltat)
  {
      // required matrices for the operations
      float sigma[3][3];
      float identity[3][3];
      identityMatrix3x3(identity);
      float skewMatrix[3][3];
      skew(skewMatrix, previousState);
      float tmp[3][3];
      // Compute the prediction covariance matrix
      scaleMatrix3x3(sigma, pow(sigmaGyro, 2), identity);
      matrixProduct3x3(tmp, skewMatrix, sigma);
      matrixProduct3x3(covariance, tmp, skewMatrix);
      scaleMatrix3x3(covariance, -pow(deltat, 2), covariance);
  }

  void KalmanFilter::getMeasurementCovariance(float covariance[3][3])
  {
      // required matrices for the operations
      float sigma[3][3];
      float identity[3][3];
      identityMatrix3x3(identity);
      float norm;
      // Compute measurement covariance
      scaleMatrix3x3(sigma, pow(sigmaAccel, 2), identity);
      vectorLength(& norm, previousAccelSensor);
      scaleAndAccumulateMatrix3x3(sigma, (1.0/3.0)*pow(ca, 2)*norm, identity);
      copyMatrix3x3(covariance, sigma);
  }

  void KalmanFilter::predictState(float predictedState[3], float gyro[3], float deltat)
  {
      // helper matrices
      float identity[3][3];
      identityMatrix3x3(identity);
      float skewFromGyro[3][3];
      skew(skewFromGyro, gyro);
      // Predict state
      scaleAndAccumulateMatrix3x3(identity, -deltat, skewFromGyro);
      matrixDotVector3x3(predictedState, identity, currentState);
      normalizeVector(predictedState);
  }

  void KalmanFilter::predictErrorCovariance(float covariance[3][3], float gyro[3], float deltat)
  {
      // required matrices
      float Q[3][3];
      float identity[3][3];
      identityMatrix3x3(identity);
      float skewFromGyro[3][3];
      skew(skewFromGyro, gyro);
      float tmp[3][3];
      float tmpTransposed[3][3];
      float tmp2[3][3];
      // predict error covariance
      getPredictionCovariance(Q, currentState, deltat);
      scaleAndAccumulateMatrix3x3(identity, -deltat, skewFromGyro);
      copyMatrix3x3(tmp, identity);
      transposeMatrix3x3(tmpTransposed, tmp);
      matrixProduct3x3(tmp2, tmp, currErrorCovariance);
      matrixProduct3x3(covariance, tmp2, tmpTransposed);
      scaleAndAccumulateMatrix3x3(covariance, 1.0, Q);
  }

  void KalmanFilter::updateGain(float gain[3][3], float errorCovariance[3][3])
  {
      // required matrices
      float R[3][3];
      float HTransposed[3][3];
      transposeMatrix3x3(HTransposed, H);
      float tmp[3][3];
      float tmp2[3][3];
      float tmp2Inverse[3][3];
      // update kalman gain
      // P.dot(H.T).dot(inv(H.dot(P).dot(H.T) + R))
      getMeasurementCovariance(R);
      matrixProduct3x3(tmp, errorCovariance, HTransposed);
      matrixProduct3x3(tmp2, H, tmp);
      scaleAndAccumulateMatrix3x3(tmp2, 1.0, R);
      invert3x3(tmp2Inverse, tmp2);
      matrixProduct3x3(gain, tmp, tmp2Inverse);
  }

  void KalmanFilter::updateState(float updatedState[3], float predictedState[3], float gain[3][3], float accel[3])
  {
      // required matrices
      float tmp[3];
      float tmp2[3];
      float measurement[3];
      scaleVector(tmp, ca, previousAccelSensor);
      subtractVectors(measurement, accel, tmp);
      // update state with measurement
      // predicted_state + K.dot(measurement - H.dot(predicted_state))
      matrixDotVector3x3(tmp, H, predictedState);
      subtractVectors(tmp, measurement, tmp);
      matrixDotVector3x3(tmp2, gain, tmp);
      sumVectors(updatedState, predictedState, tmp2);
      normalizeVector(updatedState);
  }

  void KalmanFilter::updateErrorCovariance(float covariance[3][3], float errorCovariance[3][3], float gain[3][3])
  {
      // required matrices
      float identity[3][3];
      identityMatrix3x3(identity);
      float tmp[3][3];
      float tmp2[3][3];
      // update error covariance with measurement
      matrixProduct3x3(tmp, gain, H);
      matrixProduct3x3(tmp2, tmp, errorCovariance);
      scaleAndAccumulateMatrix3x3(identity, -1.0, tmp2);
      copyMatrix3x3(covariance, tmp2);
  }


  KalmanFilter::KalmanFilter(float ca, float sigmaGyro, float sigmaAccel)
  {
      this->ca = ca;
      this->sigmaGyro = sigmaGyro;
      this->sigmaAccel = sigmaAccel;
  }

  float KalmanFilter::estimate(float gyro[3], float accel[3], float deltat)
  {
      float predictedState[3];
      float updatedState[3];
      float errorCovariance[3][3];
      float updatedErrorCovariance[3][3];
      float gain[3][3];
      float accelSensor[3];
      float tmp[3];
      float accelEarth;
      scaleVector(accel, 9.81, accel); // Scale accel readings since they are measured in gs
      // perform estimation
      // predictions
      predictState(predictedState, gyro, deltat);
      predictErrorCovariance(errorCovariance, gyro, deltat);
      // updates
      updateGain(gain, errorCovariance);
      updateState(updatedState, predictedState, gain, accel);
      updateErrorCovariance(updatedErrorCovariance, errorCovariance, gain);
      // Store required values for next iteration
      copyVector(currentState, updatedState);
      copyMatrix3x3(currErrorCovariance, updatedErrorCovariance);
      // return vertical acceleration estimate
      scaleVector(tmp, 9.81, updatedState);
      subtractVectors(accelSensor, accel, tmp);
      copyVector(previousAccelSensor, accelSensor);
      dotProductVectors(& accelEarth, accelSensor, updatedState);
      return accelEarth;
  }


  // Complementary Filter
  float ComplementaryFilter::ApplyZUPT(float accel, float vel)
  {
      // first update ZUPT array with latest estimation
      ZUPT[ZUPTIdx] = accel;
      // and move index to next slot
      uint8_t nextIndex = (ZUPTIdx + 1) % ZUPT_SIZE;
      ZUPTIdx = nextIndex;
      // Apply Zero-velocity update
      for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
          if (abs(ZUPT[k]) > accelThreshold) return vel;
      }
      return 0.0;
  }


  ComplementaryFilter::ComplementaryFilter(float sigmaAccel, float sigmaBaro, float accelThreshold)
  {
      // Compute the filter gain
      gain[0] = sqrt(2 * sigmaAccel / sigmaBaro);
      gain[1] = sigmaAccel / sigmaBaro;
      // If acceleration is below the threshold the ZUPT counter
      // will be increased
      this->accelThreshold = accelThreshold;
      // initialize zero-velocity update
      ZUPTIdx = 0;
      for (uint8_t k = 0; k < ZUPT_SIZE; ++k) {
          ZUPT[k] = 0;
      }
  }

  void ComplementaryFilter::estimate(float * velocity, float * altitude, float baroAltitude,
          float pastAltitude, float pastVelocity, float accel, float deltat)
  {
      // Apply complementary filter
      *altitude = pastAltitude + deltat*(pastVelocity + (gain[0] + gain[1]*deltat/2)*(baroAltitude-pastAltitude))+
          accel*pow(deltat, 2)/2;
      *velocity = pastVelocity + deltat*(gain[1]*(baroAltitude-pastAltitude) + accel);
      // Compute zero-velocity update
      *velocity = ApplyZUPT(accel, *velocity);
  }

  // Quaternion filter
  QuaternionFilter::QuaternionFilter(void)
  {
      q1 = 1;
      q2 = 0;
      q3 = 0;
      q4 = 0;
  }


  MadgwickQuaternionFilter::MadgwickQuaternionFilter(float beta) : QuaternionFilter::QuaternionFilter()
  {
      _beta = beta;
  }


  MadgwickQuaternionFilter6DOF::MadgwickQuaternionFilter6DOF(float beta, float zeta) :
  MadgwickQuaternionFilter::MadgwickQuaternionFilter(beta)
  {
      _zeta = zeta;
  }

  // Adapted by Simon D. Levy from
  // https://github.com/kriswiner/MPU6050/blob/master/quaternionFilter.ino
  void MadgwickQuaternionFilter6DOF::update(float ax, float ay, float az,
                                            float gx, float gy, float gz,
                                            float deltat)
  {
      static float gbiasx, gbiasy, gbiasz; // gyro bias error

      // Auxiliary variables to avoid repeated arithmetic
      float _halfq1 = 0.5f * q1;
      float _halfq2 = 0.5f * q2;
      float _halfq3 = 0.5f * q3;
      float _halfq4 = 0.5f * q4;
      float _2q1 = 2.0f * q1;
      float _2q2 = 2.0f * q2;
      float _2q3 = 2.0f * q3;
      float _2q4 = 2.0f * q4;
      //float _2q1q3 = 2.0f * q1 * q3;
      //float _2q3q4 = 2.0f * q3 * q4;

      // Normalise accelerometer measurement
      float norm = sqrt(ax * ax + ay * ay + az * az);
      if (norm == 0.0f) return; // handle NaN
      norm = 1.0f/norm;
      ax *= norm;
      ay *= norm;
      az *= norm;

      // Compute the objective function and Jacobian
      float f1 = _2q2 * q4 - _2q1 * q3 - ax;
      float f2 = _2q1 * q2 + _2q3 * q4 - ay;
      float f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
      float J_11or24 = _2q3;
      float J_12or23 = _2q4;
      float J_13or22 = _2q1;
      float J_14or21 = _2q2;
      float J_32 = 2.0f * J_14or21;
      float J_33 = 2.0f * J_11or24;

      // Compute the gradient (matrix multiplication)
      float hatDot1 = J_14or21 * f2 - J_11or24 * f1;
      float hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
      float hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
      float hatDot4 = J_14or21 * f1 + J_11or24 * f2;

      // Normalize the gradient
      norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
      hatDot1 /= norm;
      hatDot2 /= norm;
      hatDot3 /= norm;
      hatDot4 /= norm;

      // Compute estimated gyroscope biases
      float gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
      float gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
      float gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

      // Compute and remove gyroscope biases
      gbiasx += gerrx * deltat * _zeta;
      gbiasy += gerry * deltat * _zeta;
      gbiasz += gerrz * deltat * _zeta;
      gx -= gbiasx;
      gy -= gbiasy;
      gz -= gbiasz;

      // Compute the quaternion derivative
      float qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
      float qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
      float qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
      float qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

      // Compute then integrate estimated quaternion derivative
      q1 += (qDot1 -(_beta * hatDot1)) * deltat;
      q2 += (qDot2 -(_beta * hatDot2)) * deltat;
      q3 += (qDot3 -(_beta * hatDot3)) * deltat;
      q4 += (qDot4 -(_beta * hatDot4)) * deltat;

      // Normalize the quaternion
      norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
      norm = 1.0f/norm;
      q1 *= norm;
      q2 *= norm;
      q3 *= norm;
      q4 *= norm;
  }

} // namespace zestimation