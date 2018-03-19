#pragma once
#include<Eigen/Dense>

class ExtendedKalman
{
public:
	double m_dt; //The default timestep

	int m; //The number of elements in our state vector
	int n; //The number of elements in our sensor vector
	int k; //The number of elements in our control vector

	
	Eigen::VectorXd z; //The sensor vector
	Eigen::VectorXd u; //The contol vector

	Eigen::VectorXd calch(Eigen::VectorXd x, Eigen::VectorXd u); // the function expressing each sensor value in terms of the state variables
	Eigen::MatrixXd calcH(Eigen::VectorXd x, Eigen::VectorXd u); // the jacobian of the h vector wrt the state variables
	Eigen::VectorXd calcf(Eigen::VectorXd x, Eigen::VectorXd u, double dt = -1); // the physics model expressing the next state in terms of the current state and the inputs
	Eigen::MatrixXd calcF(Eigen::VectorXd x, Eigen::VectorXd u, double dt = -1); // the jacobian of the f functions wrt the state variables

	Eigen::MatrixXd V; // the process noise covariance matrix
	Eigen::MatrixXd W; // the observation noise covariance matrix

	Eigen::MatrixXd P; // the covariance matrix
	Eigen::MatrixXd K; // the kalman gain 

	double sign(double value);
	//int nextState(Eigen::VectorXd xold, Eigen::VectorXd &xnew, Eigen::VectorXd u, Eigen::VectorXd &z);

public:
	Eigen::VectorXd x; //The state vector
	ExtendedKalman(double _dt);
	int updateSensor(int index, double value);
	int updateControl(int index, double value);
	int updateState(double dt = -1.0);
};