/*
 * dubinsmaneuver.cpp - Shortest path connectint two states
 *
 *  Created on: Mar 8, 2016
 *      Author: Petr Vana
 */

#include "dubins.h"

namespace opendubins {

Dubins::Dubins(const State& newStart, const State& newEnd, const double& newRadius) {
	radius = newRadius;
	length = std::numeric_limits<double>::max();
	type = DType::Unknown;
	//std::cout << "Dubins constructor" << std::endl;
	init(newStart, newEnd);
}

void Dubins::init(State from, State to) {
	bool used = false;

	Vector dir1 = from.getNormalizedDirection();
	Vector dir2 = to.getNormalizedDirection();

	Vector dir1radius = dir1 * radius;
	Vector dir2radius = dir2 * radius;

	Point c1left = from.point + dir1radius.left();
	Point c1right = from.point + dir1radius.right();

	Point c2left = to.point + dir2radius.left();
	Point c2right = to.point + dir2radius.right();

	double n1, n2, n3, nLength, centerDistance;

	// RSR - maneuver

	Vector diff = c2right - c1right;
	double ang = diff.getAngle();

	n1 = angleToRight(from.ang, ang);
	n2 = diff.length();
	n3 = angleToRight(ang, to.ang);

	n1 = checkToleranceRight(n1);
	n3 = checkToleranceRight(n3);

	nLength = n2 + radius * (fabs(n1) + fabs(n3));

	if (nLength < length) {
		len1 = n1;
		len2 = n2;
		len3 = n3;
		length = nLength;
		used = true;
		isCCC = false;
		type = DType::RSR;
	}

	// LSL - maneuver

	diff = c2left - c1left;
	ang = diff.getAngle();

	n1 = angleToLeft(from.ang, ang);
	n2 = diff.length();
	n3 = angleToLeft(ang, to.ang);

	n1 = checkToleranceLeft(n1);
	n3 = checkToleranceLeft(n3);

	nLength = n2 + radius * (fabs(n1) + fabs(n3));

	if (nLength < length) {
		len1 = n1;
		len2 = n2;
		len3 = n3;
		length = nLength;
		used = true;
		isCCC = false;
		type = DType::LSL;
	}

	// LSR - maneuver

	diff = c2right - c1left;
	centerDistance = diff.length();

	if (centerDistance * (1.0) >= 2 * radius) {
		double alpha = asin(fmin(1, 2 * radius / centerDistance));
		double centerAngle = atan2(c2right.y - c1left.y, c2right.x - c1left.x) + alpha;
		n2 = sqrt(fmax(0, centerDistance * centerDistance - 4 * radius * radius));

		// normalize angle
		n1 = angleToLeft(from.ang, centerAngle);
		n3 = angleToRight(centerAngle, to.ang);

		n1 = checkToleranceLeft(n1);
		n3 = checkToleranceRight(n3);

		nLength = n2 + radius * (fabs(n1) + fabs(n3));

		if (nLength < length) {
			len1 = n1;
			len2 = n2;
			len3 = n3;
			length = nLength;
			used = true;
			isCCC = false;
			type = DType::LSR;
		}
	}

	// RSL - maneuver

	diff = c2left - c1right;
	centerDistance = diff.length();

	if (centerDistance * (1.0) >= 2 * radius) {
		double alpha = asin(fmin(1, 2 * radius / centerDistance));
		double centerAngle = atan2(c2left.y - c1right.y, c2left.x - c1right.x) - alpha;
		n2 = sqrt(fmax(0, centerDistance * centerDistance - 4 * radius * radius));

		// normalize angle
		n1 = angleToRight(from.ang, centerAngle);
		n3 = angleToLeft(centerAngle, to.ang);

		n1 = checkToleranceRight(n1);
		n3 = checkToleranceLeft(n3);

		nLength = n2 + radius * (fabs(n1) + fabs(n3));

		if (nLength < length) {
			len1 = n1;
			len2 = n2;
			len3 = n3;
			length = nLength;
			used = true;
			isCCC = false;
			type = DType::RSL;
		}
	}

	// CCC maneuver is possible only in case start and end state is close enougth
	if ((from.point - to.point).length() <= 4 * radius) {

		// RLR - maneuver
		diff = c2right - c1right;
		centerDistance = diff.length();

		if (centerDistance <= 4 * radius) {
			// direction of Vector(S1,S2) to Vector(S1,S3)
			double alpha = acos(centerDistance / radius / 4);

			// direction between first and second arc
			double dir12 = diff.getAngle() - M_PI / 2 - alpha;
			// direction between second and third arc
			double dir23 = diff.getAngle() + M_PI / 2 + alpha;

			n1 = angleToRight(from.ang, dir12);
			n2 = angleToLeft(dir12, dir23);
			n3 = angleToRight(dir23, to.ang);

			nLength = radius * (fabs(n1) + fabs(n2) + fabs(n3));

			if (nLength < length) {
				isCCC = true;
				len1 = n1;
				len2 = n2;
				len3 = n3;
				length = nLength;
				used = true;
				type = DType::RLR;
			}
		}

		// LRL - maneuver
		diff = c2left - c1left;
		centerDistance = diff.length();

		if (centerDistance <= 4 * radius) {
			// direction of Vector(S1,S2) to Vector(S1,S3)
			double alpha = acos(centerDistance / radius / 4);

			// direction between first and second arc
			double dir12 = diff.getAngle() + M_PI / 2 + alpha;
			// direction between second and third arc
			double dir23 = diff.getAngle() - M_PI / 2 - alpha;

			n1 = angleToLeft(from.ang, dir12);
			n2 = angleToRight(dir12, dir23);
			n3 = angleToLeft(dir23, to.ang);

			nLength = radius * (fabs(n1) + fabs(n2) + fabs(n3));

			if (nLength < length) {
				isCCC = true;
				len1 = n1;
				len2 = n2;
				len3 = n3;
				length = nLength;
				used = true;
				type = DType::LRL;
			}
		}
	}

	if (used) {
		start = from;
		end = to;
	}
}

}
