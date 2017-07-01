/*
 * Twiddle.h
 *
 *  Created on: Jun 30, 2017
 *      Author: cocoza4
 */

#ifndef SRC_TWIDDLE_H_
#define SRC_TWIDDLE_H_

#include "PID.h"
#include <iostream>
#include <vector>

using namespace std;

class Twiddle {
public:

	bool isInitialized;
	bool flag;
	bool firstFlag;
	int distance;
	int maxDistance;
	int i;
	double bestError;
	double error;
	PID pid;
	vector<double> dp;

	Twiddle(PID pid, int maxDistance);
	virtual ~Twiddle();
	void twiddle(double cte);
	bool distanceReached();

private:
	double sumDP();
};

#endif /* SRC_TWIDDLE_H_ */
