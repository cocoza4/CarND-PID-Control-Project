/*
 * Twiddle.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: cocoza4
 */

#include "Twiddle.h"

Twiddle::Twiddle(PID pid, int maxDistance) {
	this->isInitialized = false;
	this->bestError = 0;
	this->error = 0;
	this->distance = 0;
	this->maxDistance = maxDistance;
	this->i = 0;
	this->flag = true;
	this->pid = pid;
	dp = {pid.Kp/4, pid.Ki/4, 0.1};
}

Twiddle::~Twiddle() {
}

bool Twiddle::distanceReached() {
	return distance >= maxDistance;
}

double Twiddle::sumDP() {
	double sum = 0.0;
	for (auto& n : dp) {
		sum += n;
	}
	return sum;
}

void Twiddle::twiddle(double cte) {
	distance += 1;
	error += cte * cte;

	if (this->distanceReached()) {

		if (firstFlag) {
			switch (i) {
			case 0:
				pid.Kp += dp[i];
				break;
			case 1:
				pid.Ki += dp[i];
				break;
			case 2:
				pid.Kd += dp[i];
			}
		}
		double avgError = error / maxDistance;

		if (!isInitialized) {
			bestError = avgError;
			isInitialized = true;
			std::cout << "Initialization Kp: " << pid.Kp << " Ki: " << pid.Ki
									<< " Kd: " << pid.Kd << " Ki: " << pid.Ki << " Error: "
									<< bestError << " Sum dpi: " << sumDP()
									<< std::endl;
		} else {

			if (avgError < bestError) {
				bestError = avgError;
				dp[i] *= 1.1;
//				i = (i + 1) % 3;
				flag = true;
				std::cout << "New best Kp: " << pid.Kp << " Ki: " << pid.Ki
						<< " Kd: " << pid.Kd << " Ki: " << pid.Ki << " Error: "
						<< bestError << " Sum dpi: " << sumDP()
						<< std::endl;
			} else {
				if (flag) {
					flag = false;
				} else {
					switch (i) {
					case 0:
						pid.Kp += dp[i];
						break;
					case 1:
						pid.Ki += dp[i];
						break;
					case 2:
						pid.Kd += dp[i];
					}
					dp[i] *= 0.9;
//					i = (i + 1) % 3;
					flag = true;
				}
			}

			switch (i) {
			case 0:
				if (flag)
					pid.Kp += dp[i];
				else
					pid.Kp -= 2 * dp[i];
				break;
			case 1:
				if (flag)
					pid.Ki += dp[i];
				else
					pid.Ki -= 2 * dp[i];
				break;
			case 2:
				if (flag)
					pid.Kd += dp[i];
				else
					pid.Kd -= 2 * dp[i];
			}
			i = (i + 1) % 3;
		}
		std::cout << "Updated Kp: " << pid.Kp << " Ki: " << pid.Ki << " Kd: " << pid.Kd << std::endl;
		distance = 0;
		error = 0;
	}

}

