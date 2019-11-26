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

  void updateEstimate(float measured_velocity, float measured_position, float dt);

  void setControlVector(float u_vect);
  void setEstimatedCovariance(float cov[2][2]);
  void setProcessError(float err_q);
  void setSensorError(float err_r);

  float getControlVector();
  //TODO: float getEstimatedCovariance();
  float getProcessError();
  float getSensorError();

  float getInnovation();
  float getInnovationCovariance();
  float getKalmanGain();
  float getEstimateError();

  float getPosition();
  float getVelocity();

private:
  float _position;
  float _velocity;
  float _lastPosition;
  float _lastVelocity;
  float _u;
  float _initialP;
  float _Q[2][2];
  float _R[2][2];
  float _H[2][2];
  float _P[2][2];

};

#endif
