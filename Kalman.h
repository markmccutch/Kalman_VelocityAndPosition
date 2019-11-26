/*
Authored By
Mark McCutcheon, Clutch Design Solutions
Gitub: https://github.com/markmccutch
Email: mark.mccutcheon29@gmail.com
*/

#ifndef _Kalman_h_
#define _Kalman_h_

class Kalman {

public:
  Kalman(
    float position_0, float velocity_0, float U, float P_0, float Q, float R, float H
  );

  void predictEstimate(float u, int dt);
  void updateEstimate(float measured_velocity, float measured_position, int dt);

  void setControlVector(float u_vect);
  void setEstimatedCovariance(float cov[2][2]);
  void setProcessError(float err_q);
  void setSensorError(float err_r);

  float getControlVector();
  //TODO: float getEstimatedCovariance();
  //TODO: float getProcessError();
  //TODO: float getSensorError();

  //TODO: float getInnovation();
  //TODO: float getInnovationCovariance();
  //TODO: float getKalmanGain();
  //TODO: float getEstimateError();

  float getPosition();
  float getVelocity();

private:
  float _position_velocity[2];
  float _lastPosition;
  float _lastVelocity;
  float _u; //control vector (Acceleration)
  float _y; //innovation

  float _Q[2][2]; //Process Error (Accelerometer reading variance)
  float _R[2][2]; //Sensor Error (GPS reading variance)
  float _H[2][2]; //Input Translation Matrix (2X2diag(1))
  float _P[2][2]; //Estimated Covariance
  float _S[2][2]; //innovation covarience
  float _K[2][2]; //Kalman Gain

  float _A[2][2];
  float _B[2][2];

};

#endif
