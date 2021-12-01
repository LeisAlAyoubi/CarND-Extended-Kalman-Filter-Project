#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  float denominator = px*px + py*py;
  if(fabs(denominator)<=0.0001){
    std::cerr << "Error: Divison by zero.";
  }
  // compute the Jacobian matrix
  else{
    Hj << px/(sqrt(denominator)),              py/(sqrt(denominator)),                      0,                       0,
    -py/denominator,                           px/denominator,                              0,                       0,
    py*(vx*py-vy*px)/pow(denominator,3/2),     px*(vy*px-vx*py)/pow(denominator,3/2),       px/(sqrt(denominator)),  py/(sqrt(denominator));

  }

  return Hj;
}
