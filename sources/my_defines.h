/*
 * my_defines.h
 *
 *  Created on: Jun 28, 2017
 *      Author: penicrob
 */

#ifndef LP_GOP_MY_DEFINES_H_
#define LP_GOP_MY_DEFINES_H_

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <iostream>
#include "math_common.h"

#define OUTPUT_DEFAULT "\033[0m"
#define OUTPUT_BLACK "\033[30m"
#define OUTPUT_RED "\033[31m"
#define OUTPUT_GREEN "\033[32m"
#define OUTPUT_YELLOW "\033[33m"
#define OUTPUT_BLUE "\033[34m"
#define OUTPUT_MAGENTA "\033[35m"
#define OUTPUT_CYAN "\033[36m"
#define OUTPUT_WHITE "\033[37m"

#define INFO_RED(x) INFO( OUTPUT_RED << x << OUTPUT_DEFAULT )
#define INFO_YELLOW(x) INFO( OUTPUT_YELLOW << x << OUTPUT_DEFAULT )
#define INFO_MAGENTA(x) INFO( OUTPUT_MAGENTA <<  x << OUTPUT_DEFAULT )
#define INFO_CYAN(x) INFO( OUTPUT_CYAN <<  x << OUTPUT_DEFAULT )
#define INFO_GREEN(x) INFO( OUTPUT_GREEN <<  x << OUTPUT_DEFAULT )
#define INFO_WHITE(x) INFO( OUTPUT_WHITE <<  x << OUTPUT_DEFAULT )
#define INFO_BLUE(x) INFO( OUTPUT_BLUE <<  x << OUTPUT_DEFAULT )
#define INFO_BLACK(x) INFO( OUTPUT_BLACK <<  x << OUTPUT_DEFAULT )
#define INFO_COND( cond , x ) if(cond){ INFO( x ); }

#define VARIABLE_STR(s) #s
#define STR(s) VARIABLE_STR(s)
#define ROVNASE1(X) X =
#define ROVNASE(X)  ROVNASE1(X)
#define INFO_VAR(x) INFO( STR(x) << " = " <<  x )

enum ErrorType {
	NO_ERROR = 0,
	NO_START_STATE_ERROR = 1,
	NO_GOAL_STATE_ERROR = 2,
	NO_ROBOT_OBJECT_NAME_ERROR = 3,
	CAN_NOT_OPEN_FILE_ERROR = 4,
	CAN_NOT_FINE_ROBOT_NAME_S = 5,
	CAN_NOT_FINE_ROBOT_NAME_G = 6,
	NO_OUTPUT_FILE_ERROR = 7,
	NO_START_POSITION_ERROR = 8,
	NO_GOAL_POSITION_ERROR = 9,
	LAST_ERROR
};

enum PlanningState {
	state2d, state2dheading, state3d
};

typedef struct VarValue {
	std::string varName;
	int varValue;
} VarValue;

typedef struct IndexPairSOP {
	int cluster_from_node;
	int cluster_to_node;
	int cluster_from;
	int cluster_to;
} IndexPairSOP;

typedef struct IndexSOP {
	IndexSOP() {
		this->clusterIndex = -1;
		this->nodeIndex = -1;
	}

	IndexSOP(int clusterIndex_, int nodeIndex_) {
		this->clusterIndex = clusterIndex_;
		this->nodeIndex = nodeIndex_;
	}
	int clusterIndex;
	int nodeIndex;
} IndexSOP;

//typedef IloNumVarArray IloNumVarMatrix1D;
//typedef IloArray<IloNumVarArray> IloNumVarMatrix2D;
//typedef IloArray<IloArray<IloNumVarArray>> IloNumVarMatrix3D;
//typedef IloArray<IloArray<IloArray<IloNumVarArray>>> IloNumVarMatrix4D;

//typedef IloNumVarArray IloNumVarMatrix1D;
//typedef IloArray<IloNumVarArray> IloNumVarMatrix2D;
//typedef IloArray<IloArray<IloNumVarArray>> IloNumVarMatrix3D;
//typedef IloArray<IloArray<IloArray<IloNumVarArray>>> IloNumVarMatrix4D;

//typedef IloArray<IloArray<IloArray<IloNumArray>>> IloNumMatrix4D;

#define VARIABLE_FROM_TO(fromCluster,fromClusterNode,toCluster,toClusterNode)  "x_"<<fromCluster<<"_"<<fromClusterNode<<"__"<<toCluster<<"_"<<toClusterNode
#define VARIABLE_AUX_FROM_TO(fromCluster,toCluster)  "y_"<<fromCluster<<"__"<<toCluster
#define INDEX_1D( from , to , width ) ( width * from + to )
#define INDEX_2D_TO( id , width ) ( id % width )
#define INDEX_2D_FROM( id , width ) ( (id - ( id  % width )) / width )

typedef struct Point3DOriented {
	Point3DOriented() :
			x(0.0), y(0.0), z(0.0), yaw(0.0), pitch(0.0), roll(0.0) {
	}
	Point3DOriented(const double &_x, const double &_y, const double &_z, const double &_yaw, const double &_pitch, const double &_roll) :
			x(_x), y(_y), z(_z), yaw(_yaw), pitch(_pitch), roll(_roll) {
	}
	double x;
	double y;
	double z;
	double yaw;
	double pitch;
	double roll;
} Point3DOriented;

typedef struct Point3D {
	double x;
	double y;
	double z;
	Point3D() :
			x(0.0), y(0.0), z(0.0) {
	}
	Point3D(const double &_x, const double &_y, const double &_z) :
			x(_x), y(_y), z(_z) {
	}

	Point3D operator +(const Point3D &p) const {
		return Point3D(x + p.x, y + p.y, z + p.z);
	}
	Point3D operator -(const Point3D &p) const {
		return Point3D(x - p.x, y - p.y, z - p.z);
	}
	Point3D operator *(double c) const {
		return Point3D(c * x, c * y, c * z);
	}
	Point3D operator /(double c) const {
		return Point3D(x / c, y / c, z / c);
	}
	bool operator==(const Point3D &p) const {
		return ((p.x == this->x) && (p.y == this->y) && (p.z == this->z));
	}

	double distanceTo(const Point3D &p) {
		const double dfx = p.x - this->x;
		const double dfy = p.y - this->y;
		const double dfz = p.z - this->z;
		return sqrt(dfx * dfx + dfy * dfy + dfz * dfz);
	}

	void print() {
		std::cout << "[" << x << "," << y << "," << z << "," << "]" << std::endl;
	}

} Point3D;

typedef struct Point3D Vector3D;

typedef struct Face {
	unsigned int id1;
	unsigned int id2;
	unsigned int id3;
} Face;

typedef struct RotationMatrix {
	double M[3][3];
} RotationMatrix;

typedef struct TransformationMatrix {
	double M[4][4];
} TransformationMatrix;

typedef struct Position3D {
	double x;
	double y;
	double z;
	double yaw;
	double pitch;
	double roll;
	double rotationMatrix[4][4];

	Position3D() :
			x(0.0), y(0.0), z(0.0), yaw(0.0), pitch(0.0), roll(0.0) {

	}

	Position3D(double x, double y, double z, double yaw, double pitch, double roll) {
		//yaw okolo z
		//pitch okolo y
		//roll okolo x
		this->x = x;
		this->y = y;
		this->z = z;
		this->yaw = yaw;
		this->pitch = pitch;
		this->roll = roll;
	}

	void setTranslation(double x, double y, double z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	void setRotation(double yaw, double pitch, double roll) {
		this->yaw = yaw;
		this->pitch = pitch;
		this->roll = roll;
	}

	void updateRotationMatrix() {
		//XYZ rotace z http://www.songho.ca/opengl/gl_anglestoaxes.html
		//yaw okolo z
		//pitch okolo y
		//roll okolo x
		rotationMatrix[0][3] = this->x;
		rotationMatrix[1][3] = this->y;
		rotationMatrix[2][3] = this->z;
		rotationMatrix[3][3] = 1;

		rotationMatrix[0][0] = cos(this->pitch) * cos(this->yaw);
		rotationMatrix[0][1] = -cos(this->pitch) * sin(this->yaw);
		rotationMatrix[0][2] = sin(this->pitch);

		rotationMatrix[1][0] = sin(this->roll) * sin(this->pitch) * cos(this->yaw) + cos(this->roll) * sin(this->yaw);
		rotationMatrix[1][1] = -sin(this->roll) * sin(this->pitch) * sin(this->yaw) + cos(this->roll) * cos(this->yaw);
		rotationMatrix[1][2] = -sin(this->roll) * cos(this->pitch);

		rotationMatrix[2][0] = -cos(this->roll) * sin(this->pitch) * cos(this->yaw) + sin(this->roll) * sin(this->yaw);
		rotationMatrix[2][1] = cos(this->roll) * sin(this->pitch) * sin(this->yaw) + sin(this->roll) * cos(this->yaw);
		rotationMatrix[2][2] = cos(this->roll) * cos(this->pitch);

		rotationMatrix[3][0] = 0;
		rotationMatrix[3][1] = 0;
		rotationMatrix[3][2] = 0;
	}

	RotationMatrix getRotationMatrix() {
		updateRotationMatrix();
		RotationMatrix t;
		memcpy(&t.M[0][0], &(this->rotationMatrix[0][0]), 3 * sizeof(double));
		memcpy(&t.M[1][0], &(this->rotationMatrix[1][0]), 3 * sizeof(double));
		memcpy(&t.M[2][0], &(this->rotationMatrix[2][0]), 3 * sizeof(double));
		return t;
	}

	TransformationMatrix getTransformationMatrix() {
		updateRotationMatrix();
		TransformationMatrix t;
		memcpy(&t.M, this->rotationMatrix, sizeof(TransformationMatrix));
		return t;
	}

	Vector3D getTranslationVector() {
		Vector3D v(this->x, this->y, this->z);
		return v;
	}

	Position3D random(double posMIN, double posMAX, double rotMIN, double rotMAX) {
		Position3D pos;
		pos.randomFill(posMIN, posMAX, rotMIN, rotMAX);
		return pos;
	}

	void randomFill(double posMIN, double posMAX, double rotMIN, double rotMAX) {
		this->x = randDoubleMinMax(posMIN, posMAX);
		this->y = randDoubleMinMax(posMIN, posMAX);
		this->z = randDoubleMinMax(posMIN, posMAX);
		this->yaw = randDoubleMinMax(rotMIN, rotMAX);
		this->pitch = randDoubleMinMax(rotMIN, rotMAX);
		this->roll = randDoubleMinMax(rotMIN, rotMAX);
	}

	double distanceXYZ(Position3D otherPosition) {
		double diffx = otherPosition.getX() - this->x;
		double diffy = otherPosition.getY() - this->y;
		double diffz = otherPosition.getZ() - this->z;
		return sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
	}

	std::vector<double> toVector() {
		std::vector<double> vector;
		vector.push_back(this->x);
		vector.push_back(this->y);
		vector.push_back(this->z);
		vector.push_back(this->yaw);
		vector.push_back(this->pitch);
		vector.push_back(this->roll);
		return vector;
	}

	inline bool operator ==(const Position3D other) {
		if ( (getX() != other.x) || (y != other.y) || (z != other.z) || (yaw != other.yaw) || (pitch != other.pitch)
				|| (roll != other.roll) ) {
			return false;
		}
		return true;
	}

	std::ostream& operator <<(std::ostream &o) {
		o << std::fixed << std::setprecision(6) << "[" << x << "," << y << "," << z << "," << yaw << "," << pitch << ","
				<< roll << "]";
		return o;
	}

	Position3D operator-(const Position3D &other) {
		return Position3D(x - other.x, y - other.y, z - other.z, yaw - other.yaw, pitch - other.pitch,
				roll - other.roll);
	}

	void print() {
		std::cout << std::fixed << std::setprecision(6) << "[" << this->getX() << "," << this->getY() << ","
				<< this->getZ() << "," << this->getYaw() << "," << this->getPitch() << "," << this->getRoll() << "]"
				<< std::endl;
	}

	double getX() {
		return this->x;
	}
	double getY() {
		return this->y;
	}
	double getZ() {
		return this->z;
	}
	double getYaw() {
		return this->yaw;
	}
	double getPitch() {
		return this->pitch;
	}
	double getRoll() {
		return this->roll;
	}
	void setX(double x) {
		this->x = x;
	}
	void setY(double y) {
		this->y = y;
	}
	void setZ(double z) {
		this->z = z;
	}
	void setYaw(double yaw) {
		this->yaw = yaw;
	}
	void setPitch(double pitch) {
		this->pitch = pitch;
	}
	void setRoll(double roll) {
		this->roll = roll;
	}

} Position3D;

typedef struct RGBColor {
	float r;
	float g;
	float b;
	RGBColor() :
			r(0.0), g(0.0), b(0.0) {
	}
	RGBColor(const float &_r, const float &_g, const float &_b) :
			r(_r), g(_g), b(_b) {
	}

	RGBColor operator +(const RGBColor &p) const {
		return RGBColor(r + p.r, g + p.g, b + p.b);
	}
	RGBColor operator -(const RGBColor &p) const {
		return RGBColor(r - p.r, g - p.g, b - p.b);
	}
	RGBColor operator *(float c) const {
		return RGBColor(c * r, c * g, c * b);
	}
	RGBColor operator /(float c) const {
		return RGBColor(r / c, g / c, b / c);
	}
	void print() {
		std::cout << "[" << r << "," << g << "," << b << "," << "]" << std::endl;
	}

} RGBColor;

typedef struct SamplingLimits {
	double rand_max_x;
	double rand_max_y ;
	double rand_max_z ;
	double rand_min_x ;
	double rand_min_y ;
	double rand_min_z ;
}SamplingLimits;

template<typename T>
struct Plan {
	std::vector<T> plan;
	std::string name;

	void addToPlan(T position) {
		this->plan.push_back(position);
	}
};

typedef struct ellipse_rand {
	Point3DOriented point;
	double area;
	ellipse_rand(Point3DOriented point_, double area_) {
		this->point = point_;
		this->area = area_;
	}
} ellipse_rand;

//RRT SETTINGS
#define PLANNER_STATE Position3D	//rrt planner planner state
#define DIMENSION (4)				//dimension of space where rrt planner is used
#define TOPOLOGY { 1 , 1 , 1 , 2 } //topology of space
#define SCALE { 1.0 , 1.0 , 1.0  , 1.0 }
//#define MAX_DISTANCE_INCREASE 	(2.0)   //maximal distance between nodes - important not to skip collisions
#define GOAL_STATE_RANDOM_PROBABILITY_RRT (0.08) //how offten is goal state generated

std::ostream& operator <<(std::ostream &o, const Face& f);
std::ostream& operator <<(std::ostream &o, const Point3D& p);
std::ostream& operator <<(std::ostream &o, const Position3D& p);
std::ostream& operator <<(std::ostream &o, const RGBColor& p);
std::ostream& operator <<(std::ostream &o, const RotationMatrix& p);
std::ostream& operator <<(std::ostream &o, const TransformationMatrix& p);

#ifdef _WIN32
#define PATH_SEPARATOR std::string("\\")
#else
#define PATH_SEPARATOR std::string("/")
#endif

std::string getFilename (std::string& fullfilename);
std::string getPath (std::string& fullfilename);

Vector3D normalize(Vector3D &vector);
Vector3D crossProduct(Vector3D &vectorA, Vector3D	 &vectorB) ;

ellipse_rand random_point_in_ellipse(Point3D f1, Point3D f2, double a2, PlanningState planning_state,SamplingLimits sampling_limits);

#endif /* LP_GOP_MY_DEFINES_H_ */
