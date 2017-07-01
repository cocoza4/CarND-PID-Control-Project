#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	this->p_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	double error = -Kp*p_error - Kd*d_error - Ki*i_error;
	if (error < -1) {
		error = -1;
	} else if (error > 1) {
		error = 1;
	}
	return error;
}

