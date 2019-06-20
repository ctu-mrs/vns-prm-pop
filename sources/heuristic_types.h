/*
 * HeuristicTypes.h
 *
 *  Created on: Feb 22, 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_HEURISTICTYPES_H_
#define SRC_HEURISTICTYPES_H_

#include <vector>
#include <cmath>
#include <iostream>     // std::cout
#include <limits>
#include <iomanip>
#include "crl/logging.h"
#include "coords.h"
//#include <math.h>       /* isnan, sqrt */
//#include <cmath>
#include <crl/config.h>

#include "heap.h"
#include "my_defines.h"
#include "opendubins/point.h"
#include "opendubins/state.h"
#include "opendubins/dubins.h"
#include "mesh_object.h"

using namespace opendubins;

enum TSP_EDGE_WEIGHT_TYPE {
	EUC_2D, CEIL_2D, EXPLICIT
};
#define TSP_EDGE_EUC_2D_STR "EUC_2D"
#define TSP_EDGE_CEIL_2D_STR "CEIL_2D"
#define TSP_EDGE_CEIL_EXPLICIT_STR "EXPLICIT"

enum GOPTYPE {
	OP, OPN, DOP, DOPN, SOP, OBST_OP
};


enum MapFormat {
	none_format, map_points_city_points, map_file
};

template<typename T>
struct GraphNode {

	GraphNode() {
		T data;
		reward = 0;
		id = 0;
		cluster_id = -1;
	}

	GraphNode(T data_, double price_, unsigned int id_, int cluster_id_) {
		data = data_;
		reward = price_;
		id = id_;
		cluster_id = cluster_id_;
	}

	std::vector<double> toVector() {
		std::vector<double> vec = data.toVector();
		vec.push_back(reward);
		return vec;
	}

	double distanceTo(GraphNode gn1) {
		return this->data.distance(gn1.data);
	}

	T getStateInDistance(GraphNode node_to,double actualDistance){
		return this->data.getStateInDistance(node_to.data,actualDistance);
	}

	T data;
	double reward;
	unsigned int id;
	int cluster_id;
};

template<typename T>
struct Tour {
	Tour() {
		this->reward = 0;
		this->length = 0;
	}
	Tour(double reward_, double length_, std::vector<GraphNode<T>> tour_) {
		this->reward = reward_;
		this->length = length_;
		this->tour = tour_;
	}
	double reward;
	double length;
	std::vector<GraphNode<T>> tour;
	std::vector<Point> getPointTour() {
		std::vector<Point> pointTour;
		int S = tour.size();
		for ( int var = 0 ; var < S ; ++var ) {
			pointTour.push_back(tour[var].toPoint());
		}
		return pointTour;
	}
};

template<typename T>
struct DatasetOP {
	std::vector<GraphNode<T>> graph;
	double Tmax; //= available time budget per path
	unsigned int P; //= number of paths (=1)
	unsigned int startID;
	unsigned int goalID;
};


typedef struct ClusterSOP {
	int cluster_id;
	double reward;
	std::vector<int> nodeIDs;
} ClusterSOP;

template<typename T>
struct DatasetSOP {
	std::vector<ClusterSOP> clusters;
	std::vector<GraphNode<T>> nodesAll;
	std::vector<std::vector<double>> distance_matrix;
	bool is_with_nodes;
	std::string name;
	std::string type;
	std::string comment;
	TSP_EDGE_WEIGHT_TYPE edge_weight_type;
	int dimension;
	int sets;
	unsigned int startID;
	unsigned int goalID;
	double Tmax; //= available time budget per path
};

typedef struct Obstacle {
	std::vector<int> point_indexes;
} Obstacle;

enum VisibilityVertexType {
	city2map_visibility_vertex, map_visibility_vertex, city2city_visibility_vertex
};

typedef struct Visibility {
	std::vector<int> point_indexes;
	VisibilityVertexType visibility_type;
} Visibility;

typedef struct MapPoint {
	double x;
	double y;
	unsigned int id;

	Point toPoint(){
		return Point(this->x,this->y);
	}

	Coords toCoords(){
		return Coords(this->x,this->y);
	}

} MapPoint;

template<typename T>
struct DatasetOBST_OP {
	std::vector<GraphNode<T>> graph;
	double Tmax; //= available time budget per path
	unsigned int P; //= number of paths (=1)
	unsigned int startID;
	unsigned int goalID;
	std::vector<MapPoint> map_points;
	std::vector<Obstacle> obstacles;
	std::vector<Obstacle> convex_regions;
	std::vector<Visibility> visibility_lists;
	std::vector<Visibility> city2map_visibility_lists;
	std::vector<Visibility> map_visibility_lists;
	std::vector<Visibility> city2city_visibility_lists;
	std::vector<int> border;
	std::string name;
};

template<typename T>
struct DatasetOBST_OP_MeshObjects {
	double Tmax; //= available time budget per path
	unsigned int P; //= number of paths (=1)
	unsigned int startID;
	unsigned int goalID;
	std::string name;
	std::vector<MeshObject*> mesh_objects;
	std::map<std::string,double> target_reward_map;
};


typedef struct SolutionGTSP {
	std::vector<int> node_ids;
	double length;
	int num_nodes;
} SolutionGTSP;


template<typename T>
struct InsertionHistoryRecord {
	InsertionHistoryRecord(std::vector<GraphNode<T>> graph_, GraphNode<T> insertionNode_) {
		this->graph = graph_;
		this->insertionNode = insertionNode_;
	}
	std::vector<GraphNode<T>> graph;
	GraphNode<T> insertionNode;
};


typedef struct TimedImprovement {
	TimedImprovement(long timeMS_, double length_, double reward_, int iteration_) {
		this->timeMS = timeMS_;
		this->length = length_;
		this->reward = reward_;
		this->iteration = iteration_;
	}
	long timeMS;
	double length;
	double reward;
	int iteration;
} TimedImprovement;

template<typename T>
struct StartGoalNodes {
	GraphNode<T> start;
	GraphNode<T> goal;
};

typedef struct ImprovementLogRecord {
	double length;
	double reward;
	long timeMS;
	int iter;
} ImprovementLogRecord;

template<typename T>
bool operator==(const GraphNode<T>& lhs, const GraphNode<T>& rhs);

template<typename T>
std::ostream& operator <<(std::ostream &o, GraphNode<T> &p);
std::ostream& operator <<(std::ostream &o, const MapPoint& p);
//bool operator>(GraphNode &dt1, GraphNode &dt2);
//bool operator<(GraphNode &dt1, GraphNode &dt2);

typedef struct ClusterNodeDist {
	ClusterNodeDist() {
		idClusterNode = -1;
		distance = 0;
	}
	ClusterNodeDist(int idClusterNode_, double distance_) {
		idClusterNode = idClusterNode_;
		distance = distance_;
	}
	int idClusterNode;
	double distance;
} ClusterNodeDist;

typedef std::vector<std::vector<ClusterNodeDist>> shortest_matrix;
typedef std::vector<ClusterNodeDist> single_cluster_shortest_matrix;

typedef std::vector<std::vector<GraphNode<HeapPoint2D>>> samples_type_2d;
typedef std::vector<GraphNode<HeapPoint2D>> single_cluster_samples_2d;

typedef std::vector<std::vector<GraphNode<HeapPoint2DHeading>>> samples_type_2dheading;
typedef std::vector<GraphNode<HeapPoint2DHeading>> single_cluster_samples_2dheading;

typedef std::vector<std::vector<std::vector<std::vector<double>>>> samples_distances_type;
typedef std::vector<std::vector<double>> cluster_bound_distances;

template<typename T>
struct OP_Prolem {
	GOPTYPE gop_type;
	std::string name;
	double budget;
	int startIndex;
	int goalIndex;
	int oldGoalIndex = -1;
	int oldStartIndex = -1;
	int deleted_cluster_id = -1;
	bool ids_originally_from_one;
	std::vector<std::vector<GraphNode<T>>> samples;
	samples_distances_type distances;
	std::vector<MapPoint> map_points;
	std::vector<Obstacle> obstacles;
	std::vector<Obstacle> convex_regions;
	std::vector<Visibility> city2map_visibility_lists;
	std::vector<Visibility> map_visibility_lists;
	std::vector<Visibility> city2city_visibility_lists;
	std::vector<int> border;
	std::vector<MeshObject*> mesh_obstacles;
	MeshObject* mesh_robot;
	MeshObject* mesh_border;
	std::vector<HeapNode<T> *> border_nodes;
} ;


inline std::string& trim(std::string& s, const char* t = " \t\n\r\f\v") {
	s.erase(0, s.find_first_not_of(t));
	s.erase(s.find_last_not_of(t) + 1);
	return s;
}

inline std::vector<std::string> tokenize(std::string s, std::string delimiter) {
	//INFO("tokenize:\""<<s<<"\" using delimiter :\""<<delimiter<<"\"")
	std::vector<std::string> tokens;
	size_t pos = 0;
	std::string token;
	while ( (pos = s.find(delimiter)) != std::string::npos ) {
		token = s.substr(0, pos);
		//INFO("add token \""<<token<<"\"")
		tokens.push_back(token);
		s.erase(0, pos + delimiter.length());
		//INFO("s after erase\""<<s<<"\"")
	}
	if ( s.length() > 0 ) {
		//INFO("add token \""<<s<<"\"")
		tokens.push_back(s);
	}
	return tokens;
}

inline bool replace(std::string& str, const std::string& from, const std::string& to) {
	size_t start_pos = str.find(from);
	if ( start_pos == std::string::npos )
	return false;
	str.replace(start_pos, from.length(), to);
	return true;
}

TSP_EDGE_WEIGHT_TYPE parse_tsp_edge_type(std::string edge_type_string);

class HeuristicTypes {
public:
	HeuristicTypes();
	virtual ~HeuristicTypes();
};

#endif /* SRC_HEURISTICTYPES_H_ */
