#include "kalman_filter.h"
#include <iostream>

#define _USE_MATH_DEFINES

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd I;
  int x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);

  VectorXd y = z - (H_*x_);
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;
  
  // new state
  x_ = x_ + K*y;
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd I;
  int x_size = x_.size();
  I = MatrixXd::Identity(x_size, x_size);

  VectorXd z_pred;
  z_pred = VectorXd(3);

  double px = x_[0];
  double py = x_[1];
  double vx = x_[2];
  double vy = x_[3];
  double p_mag = sqrt(px*px + py*py);

  z_pred[0] = p_mag;

  if (p_mag==0){
    return;
  }

  z_pred[1] = atan2(py,px);
  z_pred[2] = (px*vx + py*vy)/p_mag;

  VectorXd y = z - z_pred;

  // Making sure phi is between -pi and pi
  while (y[1] > M_PI){
    y[1] -= 2.*M_PI;
  }

  while (y[1] < -M_PI){
    y[1] += 2.*M_PI;
  }
  
  MatrixXd Hj_t = Hj_.transpose();
  MatrixXd S = Hj_*P_*Hj_t + R_radar_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Hj_t*Si;
  
  // new state
  x_ = x_ + K*y;
  P_ = (I - K*Hj_)*P_;
}
