/*
Authored By
Mark McCutcheon, Clutch Design Solutions
Gitub: https://github.com/markmccutch
Email: mark.mccutcheon29@gmail.com
*/

#include "Kalman.h"



//Constructor
Kalman::Kalman(float position_0, float velocity_0, float U, float P_0, float Q, float R, float H) {
  _position = position_0;
  _velocity = velocity_0;
  _lastPosition = _position;
  _lastVelocity = _velocity;
  _u = U;
  _initialP = P_0;
  _Q = Q;
  _R = R;
  _H = H;

  _P[0][0] = _initialP;
  _P[0][1] = 0.0f;
  _P[1][0] = 0.0f;
  _P[1][1] = _initialP;
};


//Getters & Setters -------------------------

//setControlVector
void Kalman::setControlVector(float u_vect){
  this->_u = u_vect;
};
//getControlVector
float Kalman::getControlVector(){
  return this->_u;
};

//setEstimatedCovariance
void setEstimatedCovariance(float cov[2][2]){
  this->_P[0][0] = cov[0][0];
  this->_P[0][1] = cov[0][1];
  this->_P[1][0] = cov[1][0];
  this->_P[1][1] = cov[1][1];

};
//TODO:
//getEstimatedCovariance
//ISSUE: no direct way to return an array, and I dont think I need to waste memory on this feature

//setProcessError
void setProcessError(float err_q){
  this->_Q =
};
