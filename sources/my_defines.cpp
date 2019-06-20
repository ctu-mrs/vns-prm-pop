/*
 * my_defines.cpp
 *
 *  Created on: Jul 9 2018
 *      Author: penicrob
 */

#include "heuristic_types.h"
#include "my_defines.h"
using crl::logger;

std::ostream& operator <<(std::ostream &o, const Face& f) {
	o << "f " << f.id1 << " " << f.id2 << " " << f.id3;
	return o;
}

std::ostream& operator <<(std::ostream &o, const Point3D& p) {
	std::cout.precision(6);
	o << std::fixed << " " << std::setprecision(6) << p.x << " " << std::setprecision(6) << p.y;
	return o;
}

std::ostream& operator <<(std::ostream &o, const Position3D& p) {
	std::cout.precision(6);
	o << std::fixed << std::setprecision(6) << p.x << " " << std::setprecision(6) << p.y << " " << std::setprecision(6)
			<< p.yaw;
	return o;
}

std::ostream& operator <<(std::ostream &o, const RGBColor& c) {
	std::cout.precision(6);
	o << std::fixed << " " << std::setprecision(6) << c.r << " " << std::setprecision(6) << c.g << " "
			<< std::setprecision(6) << c.b;
	return o;
}

std::ostream& operator <<(std::ostream &o, const RotationMatrix& p) {
	o << "[" << p.M[0][0] << "," << p.M[0][1] << "," << p.M[0][2] << "," << std::endl;
	o << "[" << p.M[1][0] << "," << p.M[1][1] << "," << p.M[1][2] << "," << std::endl;
	o << "[" << p.M[2][0] << "," << p.M[2][1] << "," << p.M[2][2] << "]" << std::endl;
	return o;
}

std::ostream& operator <<(std::ostream &o, const TransformationMatrix& p) {
	o << "[" << p.M[0][0] << "," << p.M[0][1] << "," << p.M[0][2] << "," << p.M[0][3] << "," << std::endl;
	o << "[" << p.M[1][0] << "," << p.M[1][1] << "," << p.M[1][2] << "," << p.M[1][3] << "," << std::endl;
	o << "[" << p.M[2][0] << "," << p.M[2][1] << "," << p.M[2][2] << "," << p.M[2][3] << "," << std::endl;
	o << "[" << p.M[3][0] << "," << p.M[3][1] << "," << p.M[3][2] << "," << p.M[3][3] << "]" << std::endl;
	return o;
}

std::string getFilename(std::string& fullfilename) {
	unsigned found = fullfilename.find_last_of("/\\");
	std::string filename = fullfilename.substr(found + 1);
	return filename;
}

std::string getPath(std::string& fullfilename) {
	unsigned found = fullfilename.find_last_of("/\\");
	std::string path = fullfilename.substr(0, found);
	return path;
}

Vector3D normalize(Vector3D &vector) {
	Vector3D normalized;
	double norm = sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
	normalized.x /= norm;
	normalized.y /= norm;
	normalized.z /= norm;
	return normalized;
}

Vector3D crossProduct(Vector3D &vectorA, Vector3D &vectorB) {
	Vector3D product;
	product.x = vectorA.y * vectorB.z - vectorA.z * vectorB.y;
	product.y = vectorA.z * vectorB.x - vectorA.x * vectorB.z;
	product.z = vectorA.x * vectorB.y - vectorA.y * vectorB.x;
	return product;
}

/*
 Point3D random_point_in_ellipse(Point3D f1, Point3D f2, double a2) {
 a2 = 3000;
 double a = a2 / 2.0;
 double c = sqrt(pow((f1.x - f2.x), 2) + pow((f1.y - f2.y), 2)) / 2.0;
 if ( fabs(a - c) < 0.00001 ) {
 return Point3D(NAN, NAN, NAN);
 }
 double b = sqrt(a * a - c * c);
 double centerx = (f1.x + f2.x) / 2.0;
 double centery = (f1.y + f2.y) / 2.0;

 const double r = randDoubleMinMax(0, 1.0);
 const double t = M_2PI * randDoubleMinMax(0, 1.0);

 //INFO("a:" << a);
 //INFO("b:" << b);
 //INFO("c:" << c);

 //Rescale by major & minor semi-axis lengths
 double X = a * r * cos(t);
 double Y = b * r * sin(t);

 double uhel = atan2(f2.y - f1.y, f2.x - f1.x);
 //Rotate back to original coordinates

 //INFO("f1 " << f1.x << " " << f1.y)
 //INFO("f1 " << f2.x << " " << f2.y)
 const double x = (cos(uhel) * X - sin(uhel) * Y) + centerx;
 const double y = (sin(uhel) * X + cos(uhel) * Y) + centery;
 //INFO("x:" << x);
 //INFO("y:" << y);

 Point3D random_point;
 random_point.x = x;
 random_point.y = y;
 return random_point;
 }
 */

ellipse_rand random_point_in_ellipse(Point3D f1, Point3D f2, double a2, PlanningState planning_state,
		SamplingLimits sampling_limits) {
	if (planning_state == state2d || planning_state == state2dheading) {
		double a = a2 / 2.0;
		double c2 = sqrt(pow((f1.x - f2.x), 2) + pow((f1.y - f2.y), 2));
		double c = c2 / 2.0;
		if (fabs(a - c) < 0.00001) {
			return ellipse_rand(Point3DOriented(NAN, NAN, NAN, NAN, NAN, NAN), 0);
		}
		double b = sqrt(a * a - c * c);
		double centerx = (f1.x + f2.x) / 2.0;
		double centery = (f1.y + f2.y) / 2.0;
		double centerz = (f1.z + f2.z) / 2.0;

		double ellipse_area = M_PI * a * b;
		if (planning_state == state3d) {
			ellipse_area = (4.0 / 3.0) * M_PI * a * b * b;
		}

		double u = randDoubleMinMax(0, 1.0) / 4.0;

		double theta = atan(b / a * tan(M_2PI * u));
		double v = randDoubleMinMax(0, 1.0);
		double rand_theta = theta;
		if (v < 0.25) {
			rand_theta = theta;
		} else if (v < 0.5) {
			rand_theta = M_PI - theta;
		} else if (v < 0.75) {
			rand_theta = M_PI + theta;
		} else {
			rand_theta = -theta;
		}

		double max_radius = a * b / sqrt(pow(b * cos(theta), 2) + pow(a * sin(theta), 2));
		double random_radius = max_radius * sqrt(randDoubleMinMax(0, 1.0));

		double X = random_radius * cos(rand_theta);
		double Y = random_radius * sin(rand_theta);

		double uhel = atan2(f2.y - f1.y, f2.x - f1.x);
		//Rotate back to original coordinates

		const double x = (cos(uhel) * X - sin(uhel) * Y) + centerx;
		const double y = (sin(uhel) * X + cos(uhel) * Y) + centery;

		Point3DOriented random_point;
		if (planning_state == state2d) {
			random_point.x = x;
			random_point.y = y;
			random_point.z = 0;
			random_point.yaw = 0;
			random_point.pitch = 0;
			random_point.roll = 0;
		} else if (planning_state == state2dheading) {
			random_point.x = x;
			random_point.y = y;
			random_point.z = 0;
			random_point.yaw = randDoubleMinMax(0, M_2PI);
			random_point.pitch = 0;
			random_point.roll = 0;
		}
		return ellipse_rand(random_point, ellipse_area);

	} else if (planning_state == state3d) {
		double a = a2 / 2.0;
		double c2 = sqrt(pow((f1.x - f2.x), 2) + pow((f1.y - f2.y), 2) + pow((f1.z - f2.z), 2));
		double c = c2 / 2.0;
		if (fabs(a - c) < 0.00001) {
			return ellipse_rand(Point3DOriented(NAN, NAN, NAN, NAN, NAN, NAN), 0);
		}

		double centerx = (f1.x + f2.x) / 2.0;
		double centery = (f1.y + f2.y) / 2.0;
		double centerz = (f1.z + f2.z) / 2.0;

		double b = sqrt(a * a - (c * c)) / 2.0;
		//same b and c
		double ellipse_area = (4.0 / 3.0) * M_PI * a * b * b;

		double rotationz = atan2(f2.y - f1.y, f2.x - f1.x);
		double distxy = sqrt(pow(f1.x - f2.x, 2) + pow(f1.y - f2.y, 2));
		double rotationy = atan2(f2.z - f1.z, distxy);

		double phi = randDoubleMinMax(0.0, 2.0 * M_PI);
		double costheta = randDoubleMinMax(-1.0, 1.0);
		double u = randDoubleMinMax(0.0, 1.0);

		double theta = acos(costheta);

		double ra = a * cbrt(u);
		double rb = b * cbrt(u);
		double rc = b * cbrt(u);
		double x = ra * sin(theta) * cos(phi);
		double y = rb * sin(theta) * sin(phi);
		double z = rc * cos(theta);

		double x_new = cos(rotationy) * x - sin(rotationy) * z;
		double z_new = sin(rotationy) * x + cos(rotationy) * z;
		x = x_new;
		z = z_new;

		x_new = cos(rotationz) * x - sin(rotationz) * y;
		double y_new = sin(rotationz) * x + cos(rotationz) * y;
		x = x_new;
		y = y_new;

		x += centerx;
		y += centery;
		z += centerz;
		//INFO("generated point "<<x<<" "<<y<<" "<<z);
		Point3DOriented random_point;

		random_point.x = x;
		random_point.y = y;
		random_point.z = z;
		random_point.yaw = 0;
		random_point.pitch = 0;
		random_point.roll = 0;
		return ellipse_rand(random_point, ellipse_area);
	}
}

