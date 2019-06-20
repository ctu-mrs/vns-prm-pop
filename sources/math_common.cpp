/*
 * MathCommon.cpp
 *
 *  Created on: 16. 11. 2014
 *      Author: Robert Pěnička
 */

#include "math_common.h"
bool randSeeded = false;
//std::mt19937_64 generator;

//void seed(){
	//std::random_device rd;
	//generator = std::mt19937_64(rd());
	//srand(time(NULL));
	//randSeeded = true;
	//std::cout << "srand seed" << std::endl;
//}

double randDoubleMinMax(double min, double max) {
	//if (!randSeeded) {
		//srand(time(NULL));
	//	seed();
	//}
	//std::uniform_real_distribution<> dis(min, std::nextafter(max, std::numeric_limits<double>::max()));
	double random = ((double) rand() / (double) RAND_MAX);
	random = min + random * (max - min);
	//double random = dis(generator);
	return random;
}

int randIntMinMax(int min, int max) {
	//if (!randSeeded) {
	//	seed();
	//}
	//std::uniform_int_distribution<int> dis(min, max);
	int random = min + (int) (((double) rand() / ((double) RAND_MAX + 1)) * (max - min + 1));
	//int random = dis(generator);
	return random;
}

double normalizeAngle(double angle, double min, double max) {
	double normalized = angle;
	while (normalized < min) {
		normalized += M_2PI;
	}
	while (normalized > max) {
		normalized -= M_2PI;
	}
	return normalized;
}

int sgn(int val) {
	if (val > 0) {
		return 1;
	} else if (val < 0) {
		return -1;
	} else {
		return 0;
	}
}

double sgn(double val) {
	if (val > 0) {
		return 1;
	} else if (val < 0) {
		return -1;
	} else {
		return 0;
	}
}

float sgn(float val) {
	if (val > 0) {
		return 1;
	} else if (val < 0) {
		return -1;
	} else {
		return 0;
	}
}

std::vector<double> range(double min, double max, double step) {
	std::vector<double> range_values;
	if (min <= max) {
		for (double val = 0; val < max; val = val + step) {
			range_values.push_back(val);
		}
		range_values.push_back(max);
	}
	return range_values;
}


