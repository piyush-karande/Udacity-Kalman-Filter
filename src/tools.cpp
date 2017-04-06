#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if (estimations.size()==0 || estimations.size() != ground_truth.size()){
  	std::cout << "CalculateRMSE - Error!! estimations and ground_truth dont match";
  	return rmse;
  }

  for (int i=0; i<estimations.size(); i++){
  	VectorXd residual = estimations[i] - ground_truth[i];

  	residual = residual.array()*residual.array();
  	rmse += residual;
  }

  rmse = rmse/estimations.size();

  rmse = rmse.array().sqrt();

  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  //recover state parameters
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double sqr_sum = px * px + py * py;

  if (sqr_sum == 0) {
    std::cout << "CalculateJacobian() - Error - Division by Zero";
    return Hj;
  } 
  else {
    double sqr_sum_half = pow(sqr_sum, 0.5);
    double sqr_sum_three_half = pow(sqr_sum, 1.5);
    
    Hj(0, 0) = px / sqr_sum_half;
    Hj(0, 1) = py / sqr_sum_half;
    Hj(1, 0) = -py / sqr_sum;
    Hj(1, 1) = px / sqr_sum;
    Hj(2, 0) = py * (vx * py - vy * px) / sqr_sum_three_half;
    Hj(2, 1) = px * (vy * px - vx * py) / sqr_sum_three_half;
    Hj(2, 2) = px / sqr_sum_half;
    Hj(2, 3) = py / sqr_sum_half;
  }
  //compute the Jacobian matrix

  return Hj;
}
