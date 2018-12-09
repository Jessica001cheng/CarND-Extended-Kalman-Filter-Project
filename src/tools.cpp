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
 VectorXd count(4);
 count << 0,0,0,0;
 if(estimations.size() == 0 || estimations.size() != ground_truth.size())
 {
   cout << "input size is wrong, return" << endl;
   return rmse; 
 }
 for(int i=0;i<estimations.size();i++)
 {
   count = estimations[i].array() - ground_truth[i].array();
   count = count.array()*count.array();
   rmse += count; 
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
  MatrixXd jac = MatrixXd(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float psqr = px*px + py*py;
  float psrt = sqrt(psqr);

	//check division by zero
	if(fabs(psqr) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return jac;
	}

  jac << px/psrt, py/psrt, 0,0,
         -py/psqr, px/psqr, 0 ,0,
         py*(vx*py - vy*px)/(psrt*psqr), px*(vy*px - vx*py)/(psrt*psqr), px/psrt, py/psrt;

  return jac;

}
