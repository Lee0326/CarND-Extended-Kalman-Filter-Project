#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // state vector
  P_ = P_in; // covariance matrix
  F_ = F_in; // state transition matrix
  H_ = H_in; // measurement matrix
  R_ = R_in; // measurement covarianve matrix
  Q_ = Q_in; // process covatiance 
}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred; // calculate error
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_* Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// new estimate 
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);  
	double rho = sqrt(px*px + py*py);
	if (rho < 0.000001) {
      rho = 0.000001;
    }
	double phi = atan2(py , px); // make sure it is in -pi to pi
	double rhodot = (px*vx + py*vy) / rho;
	VectorXd h = VectorXd(3);
	h << rho, phi, rhodot;
	VectorXd y = z - h;
   float PI = 3.1415926;
  // nomoliza the phi data
   while(y(1) > PI){
    y(1) -= (2*PI);
  }

  while(y(1) < -PI){
    y(1) += (2*PI);
  }
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_* Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
