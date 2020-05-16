#pragma once
#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

	// state vector
	Eigen::VectorXd x_;

	// state covariance matrix
	Eigen::MatrixXd P_;

	// state transistion matrix
	Eigen::MatrixXd F_;

	// process covariance matrix
	Eigen::MatrixXd Q_;

	// measurement matrix
	Eigen::MatrixXd H_;

	// measurement covariance matrix
	Eigen::MatrixXd R_;

	/*
	Constructor
	*/
	KalmanFilter();

	/*
	Destructor
	*/
	virtual ~KalmanFilter();

	/*
	* Init Initializes Kalman filter
	* x_in: initial state
	* P_in: initial state covariance
	* F_in: Transition matrix
	* H_in: Measurement matrix 
	* R_in: Measurement covariance matrix 
	* Q_in: Process covariance matrix
	*/
	void Init(Eigen::VectorXd& x_in, Eigen::MatrixXd& P_in, Eigen::MatrixXd& F_in,
		Eigen::MatrixXd& H_in, Eigen::MatrixXd& R_in, Eigen::MatrixXd& Q_in);

	/*
	* Prediction predict the state and the state covariance
	* using the process model
	* delta_T: time between k and k+1 in s
	*/
	void Predict();

	/*
	* updates the state by using Extended Kalman Filter equations
	* z: the measurement at k+1
	*/
	void Update(const Eigen::VectorXd& z);

	/*
	* updates the state by using Extended Kalman Filter equations
	* z: the measurement at k+1
	*/
	void UpdateEKF(const Eigen::VectorXd& z);
};

#endif // !KALMAN_FILTER_H_

