/*
Authored By
Mark McCutcheon, Clutch Design Solutions
Gitub: https://github.com/markmccutch
Email: mark.mccutcheon29@gmail.com

Dependencies:
MatrixMath: https://github.com/eecharlie/MatrixMath

*/

#include "Kalman.h"
#include "MatrixMath.h"


//Constructor
Kalman::Kalman(float position_0, float velocity_0, float P_0, float Q, float R, float H) {
  _position_velocity[0] = position_0;
  _position_velocity[1] = velocity_0;
  _lastPosition = position_0;
  _lastVelocity = velocity_0;
  _u = 0;
  _y = 0;

  //fill Matrices with diagonal matrices with the required values
  for (int i =0; i<2; i++) {
    for (int j = 0; j<2; j++) {
      if( i == j) {
        _Q[i][j] = Q;
        _R[i][j] = R;
        _H[i][j] = H;
        _P[i][j] = P_0;
        _S[i][j] = 0;
        _K[i][j] = 0;
      } else {
        _Q[i][j] = 0;
        _R[i][j] = 0;
        _H[i][j] = 0;
        _P[i][j] = 0;
        _S[i][j] = 0;
        _K[i][j] = 0;
      }
    }
  }
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
  this->_Q[0][0] = err_q;
  this->_Q[0][1] = 0;
  this->_Q[1][0] = 0;
  this->_Q[1][1] = err_q;
};

//TODO: getProcessError

//setSensorError
void setSensorError(float err_r){
  this->_R[0][0] = err_r;
  this->_R[0][1] = 0;
  this->_R[1][0] = 0;
  this->_R[1][1] = err_r;
};

//TODO: getSensorError

//getInnovation
float getInnovation(){
   return this->_y;
};

//TODO: getInnovationCovariance

//TODO: getKalmanGain

//TODO: getEstimateError

//getPosition
float getPosition() {
  return this->_position_velocity[0];
}

//getVelocity
float getVelocity(){
  return this->_position_velocity[1];
}


// Calculations -------------------------

//updateEstimate
void predictEstimate(float u, int dt){

  //Transfer state to old state storage
  _lastPosition = _position_velocity[0];
  _lastVelocity = _position_velocity[1];
  //PREDICT
  //maintain values
  _position = _lastPosition + _lastVelocity*dt + 0.5*dt*dt*_u;
  _velocity = _lastVelocity + dt*_u;
  //maintain covariance




}
