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
void setEstimatedCovariance()
