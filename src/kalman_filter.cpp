#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
  x_= F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft+Q_; 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z- H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ *Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ *Ht *Si;
  
  
  x_ = x_ + (K *y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I-K*H_) *P_;
 
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float x = x_(0);
  float y_dir = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  //std::cout << "x " << x << std::endl;
  //std::cout << "y " << y_dir << std::endl;
  //std::cout << "vx " << vx << std::endl;
  //std::cout << "vy " << vy << std::endl;
  

  float rho = sqrt(x*x + y_dir*y_dir);
  float theta = atan2(y_dir,x);
  float ro_dot = ((x*vx)+(y_dir*vy))/rho;
  
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;
  
  VectorXd  y = z-z_pred;
  
  
  while (y(1) > M_PI)
  {
    std::cout << "Above" << y(1) << std::endl;
    y(1) -= 2*M_PI;
  }
  
  while (y(1) < -M_PI)
  {
    std::cout << "below" << y(1) << std::endl;
    y(1) += 2*M_PI;

  }
  std::cout << "after " << y(1) << std::endl;
    
  MatrixXd Ht = H_.transpose();   
  MatrixXd S = H_ * P_ *Ht +R_ ;   
  MatrixXd Si = S.inverse();    
  MatrixXd K = P_*Ht *Si;    
  
  x_ = x_ + (K *y);
  
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I-K*H_) *P_;
  
}
