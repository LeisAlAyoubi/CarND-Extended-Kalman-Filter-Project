#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if((estimations.size()==0) || (estimations.size() != ground_truth.size())){
    std::cout<< "Error: Invalid estimation or ground truth data!" << std::endl;
  }

  else{
  // TODO: accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    VectorXd residual_squared = residual.array() * residual.array();
    
    rmse += residual_squared;
  }

  // TODO: calculate the mean
  rmse /= estimations.size();

  // TODO: calculate the squared root
  rmse = rmse.array().sqrt();
  // return the result
  }
  return rmse;
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
