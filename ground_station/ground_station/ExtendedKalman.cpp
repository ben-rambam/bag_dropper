#define _USE_MATH_DEFINES

#include "ExtendedKalman.h"
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

/*
int ExtendedKalman::nextState(VectorXd xold, VectorXd &xnew, VectorXd u, VectorXd &z)
{
	//assign our control variables
	double vpa = u[0];
	double phi = u[1];
	double theta = u[2];

	//calculate and set the ground speeds of the aircraft based on controlled vpa


}
*/
double ExtendedKalman::sign(double val)
{
	if (val >= 0)
		return 1.0;
	else
		return -1.0;
}

VectorXd ExtendedKalman::calch(VectorXd x, VectorXd u) // the function expressing each sensor value in terms of the state variables
{
	VectorXd h = VectorXd::Zero(n);

	h(0) = x(0); // x position from GPS
	h(1) = x(1); // y position from GPS
	h(2) = tan(x(4) / x(3)); // heading from GPS
	h(3) = sqrt(pow(x(3), 2) + pow(x(4), 2)); // speed from GPS
	h(4) = x(2); // altitude from GPS
	h(5) = x(2); // altitude from barometer
	h(6) = sqrt(pow(x(3) - x(6), 2) + pow(x(4) - x(7), 2) + pow(x(5), 2)); // airspeed from pitot tube
	h(7) = x(8); // roll of the aircraft from IMU
	h(8) = x(9); // pitch of the aircraft from IMU
	h(9) = tan((x(4) - x(7)) / (x(3) - x(6))); // yaw of the aircraft from IMU
	h(10) = M_PI_2 - tan((x(4) - x(7)) / (x(3) - x(6))) - tan((x(11) - x(1)) / (x(10) - x(0))); // yaw angle to target from gimbal
	h(11) = tan(x(2) / sqrt(pow(x(11) - x(1), 2) + pow(x(10) - x(0), 2))); //pitch angle to target from gimbal

	return h;
}
MatrixXd ExtendedKalman::calcH(VectorXd x, VectorXd u) // the jacobian of the h vector wrt the state variables
{
	MatrixXd H = MatrixXd::Zero(m, n);

	//numerically approximate the jacobian by perturbing each state variable in turn and storing the results
	//calculate our baseline
	VectorXd baseline(n);
	VectorXd perturbed(n);
	baseline = calch(x, u);
	//for each state variable
	for (int i = 0; i < n; i++)
	{
		//set the step value;
		double step = 0.000001;
		//perturb the relevant state variable;
		x(i) += step;
		//calculate the physics with the perturbation
		perturbed = calch(x, u);
		//store the finite difference derivative in the appropriate column of F
		H.col(i) = (perturbed - baseline) / step;
		//restore the state variable
		x(i) -= step;
	}

	return H;
}
VectorXd ExtendedKalman::calcf(VectorXd x, VectorXd u, double dt) // the physics model expressing the next state in terms of the current state and the inputs
{
	if (dt < 0)
		dt = m_dt;

	cout << "m: " << m << endl;
	VectorXd f(m);
	f = VectorXd::Zero(m);
	double k1 = 1.0/2.0 *1.225 * 1.2 * 5.5 / (30 / 2.2);
	cout << "k1: " << k1 << endl;
	double a0L = -4.0*M_PI / 180.0;
	double g = 9.81; //acceleration due to gravity in m/s^2

					 //double psi = atan2(x(4), x(3)); // yaw wrt north
	double vpa = sqrt(pow(x(3) - x(6), 2) + pow(x(4) - x(7), 2) + pow(x(5), 2)); //airspeed. needed to calculate lift
	cout << "vpa: " << vpa << endl;
	double ay = k1*vpa*vpa * (x(9) - a0L - asin(x(5) / vpa))*sin(x(8)); // horizontal radial acceleration due to lift, positive right
	cout << "ay: " << ay << endl;
	double az = -k1*vpa*vpa*(x(9) - a0L - asin(x(5) / vpa))*cos(x(8)) + g; // vertical radial accelration due to lift, positive down
	cout << "az: " << az << endl;
	double cospsi = 1;
	double sinpsi = 0;
	if (abs(x(3)) > 0.01 || abs(x(4)) > 0.01)
	{
		cospsi = x(4) / sqrt(x(3)*x(3) + x(4)*x(4));
		sinpsi = x(3) / sqrt(x(3)*x(3) + x(4)*x(4));
	}

	cout << "cospsi: " << cospsi << endl;
	cout << "sinpsi: " << sinpsi << endl;


	f(0) = x(0) + x(3)*dt + 0.5*ay*dt*dt*cospsi;
	f(1) = x(1) + x(4)*dt - 0.5*ay*dt*dt*sinpsi;
	f(2) = x(2) + x(5)*dt - 0.5*az*dt*dt;
	f(3) = x(3) + ay*cospsi*dt;
	f(4) = x(4) - ay*sinpsi*dt;
	f(5) = x(5) - az*dt;
	f(6) = x(6);
	f(7) = x(7);
	f(8) = x(8) + u(0);
	f(9) = x(9) + u(1);
	f(10) = x(10);
	f(11) = x(11);

	return f;
}
MatrixXd ExtendedKalman::calcF(VectorXd x, VectorXd u, double dt) // the jacobian of the f functions wrt the state variables
{
	if (dt <= 0)
		dt = m_dt;

	MatrixXd F = MatrixXd::Zero(n, n);

	//numerically approximate the jacobian by perturbing each state variable in turn and storing the results
	//calculate our baseline
	VectorXd baseline(n);
	VectorXd perturbed(n);
	baseline = calcf(x, u, dt);
	//for each state variable
	for (int i = 0; i < n; i++)
	{
		//set the step value;
		double step = 0.000001;
		//perturb the relevant state variable;
		x(i) += step;
		//calculate the physics with the perturbation
		perturbed = calcf(x, u, dt);
		//store the finite difference derivative in the appropriate column of F
		F.col(i) = (perturbed - baseline) / step;
		//restore the state variable
		x(i) -= step;
	}

	return F;
}


/***************************************************************************
* This is the constructor. For this class, it doesn't do much because
* everything in the class has been fully defined.
***************************************************************************/
ExtendedKalman::ExtendedKalman(double _dt)
{
	m = 12;
	n = 12;
	k = 3;
	if (_dt > 0)
		m_dt = _dt;
	else
		throw - 1;

	V = MatrixXd::Zero(n, n); // Zero out the process noise matrix
							  // Set the relevant variables in the process noise matrix
	V(0, 0) = 0.2;
	V(1, 1) = 0.2;
	V(2, 2) = 0.05;
	V(3, 3) = 0.5;
	V(4, 4) = 0.5;
	V(5, 5) = 0.25;
	V(6, 6) = 2.0;
	V(7, 7) = 2.0;
	V(8, 8) = 0.005;
	V(9, 9) = 0.005;
	V(10, 10) = 1;
	V(11, 11) = 1;

	W = MatrixXd::Zero(m, m); // Zero out the observation noise matrix
							  // Set the relevant variables in the observation noise matrix
	W(0, 0) = 0.2;
	W(1, 1) = 0.2;
	W(2, 2) = 0.01;
	W(3, 3) = 1.0;
	W(4, 4) = 0.3;
	W(5, 5) = 0.5;
	W(6, 6) = 2.0;
	W(7, 7) = 0.005;
	W(8, 8) = 0.005;
	W(9, 9) = 0.005;
	W(10, 10) = 0.01;
	W(11, 11) = 0.01;
	
	P = MatrixXd::Ones(n, n) * 10000.0; // Set the covariance matrix to large numbers
	K = MatrixXd::Zero(n, m); // Zero out the kalman gains

							  //initialize our state estimate
	x = VectorXd::Zero(n);
	u = VectorXd::Zero(k);
}

/***************************************************************************
* This function should be called whenever a sensor provides additional data.
***************************************************************************/
int ExtendedKalman::updateSensor(int index, double value)
{
	z(index, 1) = value;
	return 0;
}

/***************************************************************************
* This function should be called whenever a control input is sent
***************************************************************************/
int ExtendedKalman::updateControl(int index, double value)
{
	u(index, 1) = value;
	return 0;
}

/***************************************************************************
* This function updates the state of the Kalman filter. It should be called
* as frequently as possible. If the time step is varying from
* call to call, then pass dt in as a parameter. If not, the last value input
* will be used. This could be the value from the constructor.
**************************************************************************/
int ExtendedKalman::updateState(double _dt)
{
	cout << "updating state:" << endl;
	if (_dt > 0)
	{
		m_dt = _dt;
	}

	MatrixXd F(n, n);
	MatrixXd H(m, n);

	//xnew = f(xold,uold)
	x = calcf(x, u, m_dt);
	//Pnew = F(xnew,uold)*Pold*F(xnew,uold)' + V
	F = calcF(x, u, m_dt);
	P = F*P*F.transpose() + V;
	//K = Pnew*H(xnew,uold)'*(Hnew*Pnew*Hnew'+W)^-1
	H = calcH(x, u);
	K = (H * P * H.transpose() + W).transpose().colPivHouseholderQr().solve(H*P.transpose()).transpose();
	//xnew = xnew + Knew*(znew-h(xnew))
	x = x + K*(z - calch(x, u));
	cout << "updated x: \n" << x << endl;
	//Pnew = (I-Knew*Hnew)*Pnew
	P = (MatrixXd::Identity(n, n) - K*H)*P;

	return 0;
}