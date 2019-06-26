/*
 * PRM.h
 *
 *  Created on: 1. 2. 2015
 *      Author: Robert Pěnička
 */

#ifndef SRC_PLANNER_PRM_H_
#define SRC_PLANNER_PRM_H_
#include "crl/logging.h"
#include "my_defines.h"
#include <vector>
#include "math_common.h"
#include <algorithm>
#include <crl/config.h>
#include <limits>

#include "opendubins/dubins.h"
#include <flann/flann.hpp>

#include "mesh_object.h"
#include "tree_node.h"
//#include "map_point.h"
#include "dijkstra.h"
#include <crl/alg/algorithm.h>
#include "crl/gui/canvas.h"
#include <crl/gui/shape.h>

using namespace crl::gui;
using namespace opendubins;
using crl::logger;

#define OUTPUT_AFTER_ADDED_NUM_PRM 100

template<class T>
class PRM {
public:
	PRM(crl::CConfig& config, PlanningState planning_state_);
	virtual ~PRM();
	void create_initial_graph(std::vector<MeshObject*> &obstacles_, MeshObject* robot_,
			std::vector<HeapNode<T>*> &cities_nodes_, int initial_size);
	void add_uniform_points(int num_points_to_add);

	bool add_point(Point3DOriented point);
	void calculate_added_points(bool add_points = true);
	std::vector<HeapNode<T>*>& get_points();

	plan_with_length<T> plan(int start_index, int goal_index);
	std::vector<plan_with_length<T>> plan(int start_index, std::vector<int> goal_indexes);
	void set_borders(std::vector<HeapNode<HeapPoint3D> *> borders);
	void set_gui(CCanvasBase* canvas_);
	void set_num_headings(int num_headings);
private:
	//void randomState(MapPoint<PLANNER_STATE>* node);

	// FILL RANDUM STATE
	void fillRandomState(HeapNode<T> * positionToFill);
	void fillPoint(HeapNode<T>* newNode, Point3DOriented point);

	//TEST POINTS BETWEEN STATES
	bool isPathFreeBetweenNodes(HeapNode<T> * actual, HeapNode<T> * neigbour, std::vector<MeshObject*> & obstacles,
			MeshObject* robot);
	//MPNN::ANNpoint createANNpoint(MapPoint<PLANNER_STATE>* node);
	//MPNN::ANNpoint fillANNpoint(MPNN::ANNpoint aNNpoint, MapPoint<PLANNER_STATE>* treeNode);
	//FILL OR CREATE ANN POINT
	//MPNN::ANNpoint fillANNpoint(HeapNode<T> * heepNode, MPNN::ANNpoint aNNpoint = NULL);

	//test collision
	bool testCollision(std::vector<MeshObject*> obstacles, MeshObject* object, Position3D object_position);
	bool testCollision(std::vector<MeshObject*> obstacles, MeshObject* object, HeapNode<T>* node);

	void draw_points(std::vector<HeapNode<T> *> &points, CShape* color = NULL, bool clear_actual = true);
	void draw_connections(std::vector<HeapNode<T> *> &points, CShape* color = NULL, bool clear_actual = true);

	flann::Index<flann::L2<float>> *flann_index;
	flann::Index<flann::L2Dubins<float>> *flann_index_dub;
	int filter_nn(HeapNode<T>* actual, std::vector<std::vector<int>> & indices,
			std::vector<std::vector<int>> & indices_rew, int generateIndex, int k, int k_search);

	//MPNN::MultiANN<HeapNode*> *kdTree;
	//int *topology;
	//MPNN::ANNcoord *scale;
	CCanvasBase* canvas;
	Dijkstra<T> dijskra;

	std::vector<MeshObject*> obstacles;
	MeshObject* robot;
	std::vector<HeapNode<T> *> cities_nodes;
	//int generated_number_of_neighbors;

	std::vector<HeapNode<T> *> generatedFreePositions;
	double rand_min_x;
	double rand_max_x;
	double rand_min_y;
	double rand_max_y;
	double rand_min_z;
	double rand_max_z;

	PlanningState planning_state;
	double dubins_radius;
	int num_headings;

	double collision_distance_check;

	bool continuous_point_adding;
	int continuous_point_adding_start_id;

};

template<class T>
PRM<T>::PRM(crl::CConfig& config, PlanningState planning_state_) {
	// TODO Auto-generated constructor stub
	continuous_point_adding = false;
	continuous_point_adding_start_id = 0;
	planning_state = planning_state_;
	num_headings = 0;
	this->dubins_radius = config.get<double>("dubins-radius");
	this->collision_distance_check = config.get<double>("collision-distance-check");
	if (collision_distance_check == 0) {
		ERROR("you need to specify collision_distance_check for sampling-based motion planning");
		exit(1);
	}
}

template<class T>
PRM<T>::~PRM() {
	// TODO Auto-generated destructor stub
}

template<class T>
void PRM<T>::create_initial_graph(std::vector<MeshObject*> &obstacles_, MeshObject* robot_,
		std::vector<HeapNode<T>*> &cities_nodes_, int initial_size) {
	INFO("generate graph");
	//generated_number_of_neighbors = NUMBER_OF_NEAREST_NEIGBOURS + 1;
	this->cities_nodes = cities_nodes_;
	this->robot = robot_;
	this->obstacles = obstacles_;

	int generateIndex = 0;
	int addedCounter = 0;
	int collisionCounter = 0;
	int collisionCounterTot = 0;
	bool collisionDetected = true;

//add also cities
	generatedFreePositions.reserve(initial_size + cities_nodes.size());
	INFO("reserved");

//try to connect goal
	int citiesNumConnectionsAdded = 0;
	INFO("we have "<<cities_nodes.size()<<"city nodes")
	for (int cityIndex = 0; cityIndex < cities_nodes.size(); ++cityIndex) {
		//MPNN::ANNpoint newPoint = fillANNpoint(cities_nodes[cityIndex]);
		INFO("city: "<<*cities_nodes[cityIndex]);
		if (!testCollision(obstacles, robot, cities_nodes[cityIndex])) {
			//kdTree->AddPoint(newPoint, cities_nodes[cityIndex]);
			generatedFreePositions.push_back(cities_nodes[cityIndex]);
		} else {
			ERROR("collision detected in city with index "<<cityIndex<<"!!!! exiting....");
			exit(1);
		}

	}
	INFO("added " << generatedFreePositions.size() << " cities positions");
	draw_points(generatedFreePositions);

	double next_initial_sampling_info = 0.0;
//generate NUM_GENERATE_POSITIONS_AT_ONCE points
	for (generateIndex = 0; generateIndex < initial_size; ++generateIndex) {
		collisionDetected = true;
		HeapNode<T>* newNode = new HeapNode<T>();
		//INFO("generated "<<generateIndex);
		newNode->node_id = generatedFreePositions.size();	//start and goal are 0 and 1
		newNode->cluster_id = newNode->node_id;	//start and goal are 0 and 1
		newNode->city_node = false;
		while (collisionDetected) {
			fillRandomState(newNode);
			if (!testCollision(obstacles, robot, newNode)) {
				generatedFreePositions.push_back(newNode);
				collisionDetected = false;
				addedCounter++;
				//output every OUTPUT_AFTER_ADDED_NUM_PRM generated free positions
				if (((double) addedCounter) / ((double) initial_size) >= next_initial_sampling_info) {
					next_initial_sampling_info += 0.1;
					//INFO(	"collision states "<<(100.0*((double)collisionCounter)/((double) (collisionCounter+addedCounter)))<<"%");
					INFO("added "<< addedCounter <<" configurations out of "<<initial_size);
					draw_points(generatedFreePositions);
				}
			} else {
				//INFO("collision detected");
				collisionDetected = true;
				collisionCounter++;
				collisionCounterTot++;
			}
		}
	}

	INFO("testing flann");
	if (planning_state == state2d) {
		flann::Matrix<float> new_points_matrix(new float[generatedFreePositions.size() * 2],
				generatedFreePositions.size(), 2);
		for (int var = 0; var < generatedFreePositions.size(); ++var) {
			new_points_matrix[var][0] = generatedFreePositions[var]->data.x;
			new_points_matrix[var][1] = generatedFreePositions[var]->data.y;
		}
		flann_index = new flann::Index<flann::L2<float>>(new_points_matrix, flann::KDTreeIndexParams(4));
		flann_index->buildIndex();
	} else if (planning_state == state2dheading) {
		flann::Matrix<float> new_points_matrix(new float[generatedFreePositions.size() * 3],
				generatedFreePositions.size(), 3);
		for (int var = 0; var < generatedFreePositions.size(); ++var) {
			new_points_matrix[var][0] = generatedFreePositions[var]->data.x;
			new_points_matrix[var][1] = generatedFreePositions[var]->data.y;
			new_points_matrix[var][2] = ((HeapNode<HeapPoint2DHeading>*) generatedFreePositions[var])->data.phi;
		}
		flann::L2Dubins<float>::radius = this->dubins_radius;
		flann_index_dub = new flann::Index<flann::L2Dubins<float>>(new_points_matrix, flann::KDTreeIndexParams(4));
		flann_index_dub->buildIndex();
	} else if (planning_state == state3d) {
		flann::Matrix<float> new_points_matrix(new float[generatedFreePositions.size() * 3],
				generatedFreePositions.size(), 3);
		for (int var = 0; var < generatedFreePositions.size(); ++var) {
			new_points_matrix[var][0] = generatedFreePositions[var]->data.x;
			new_points_matrix[var][1] = generatedFreePositions[var]->data.y;
			new_points_matrix[var][2] = ((HeapNode<HeapPoint3D>*)  generatedFreePositions[var])->data.z;
		}
		flann_index = new flann::Index<flann::L2<float>>(new_points_matrix, flann::KDTreeIndexParams(4));
		flann_index->buildIndex();
	}

	//int nn = generated_number_of_neighbors;

	//float sd = 2; //configuration space dimension
	//float num_points = generatedFreePositions.size();

	draw_points(generatedFreePositions);
	INFO("generated " << initial_size << " random positions");
	INFO("collisionCounterTot " << collisionCounterTot << " positions");
	continuous_point_adding_start_id = 0;
	this->continuous_point_adding = true;
	calculate_added_points(false);
	INFO("end create_initial_graph");
}

template<class T>
void PRM<T>::add_uniform_points(int num_points_to_add) {
	if (!this->continuous_point_adding) {
		this->continuous_point_adding = true;
		this->continuous_point_adding_start_id = generatedFreePositions.size();
	}
	int generateIndex = 0;
	int addedCounter = 0;
	int collisionCounter = 0;
	int collisionCounterTot = 0;
	bool collisionDetected = true;
	for (generateIndex = 0; generateIndex < num_points_to_add; ++generateIndex) {
		collisionDetected = true;
		HeapNode<T>* newNode = new HeapNode<T>();
		//INFO("generated "<<generateIndex);
		newNode->node_id = generatedFreePositions.size();	//start and goal are 0 and 1
		newNode->cluster_id = newNode->node_id;	//start and goal are 0 and 1
		newNode->city_node = false;
		while (collisionDetected) {
			fillRandomState(newNode);
			if (!testCollision(obstacles, robot, newNode)) {
				generatedFreePositions.push_back(newNode);
				collisionDetected = false;
				addedCounter++;

			} else {
				//INFO("collision detected");
				collisionDetected = true;
				collisionCounter++;
				collisionCounterTot++;
			}
		}
	}
}

template<class T>
bool PRM<T>::add_point(Point3DOriented point) {
	if (!this->continuous_point_adding) {
		this->continuous_point_adding = true;
		this->continuous_point_adding_start_id = generatedFreePositions.size();
	}
	bool point_added = false;
	HeapNode<T>* newNode = new HeapNode<T>();
	newNode->node_id = generatedFreePositions.size();	//start and goal are 0 and 1
	newNode->cluster_id = generatedFreePositions.size();
	newNode->city_node = false;
	fillPoint(newNode, point);

	//INFO("test collision");
	if (!testCollision(this->obstacles, this->robot, newNode)) {
		//MPNN::ANNpoint newPoint = fillANNpoint(newNode);

		//kdTree->AddPoint(newPoint, newNode);
		generatedFreePositions.push_back(newNode);
		//INFO("new point created, points size "<<generatedFreePositions.size()<<" flann_index.size() " <<flann_index->size()<<" continuous_point_adding_start_id "<<continuous_point_adding_start_id);
		point_added = true;
	} else {
		delete newNode;
	}
	return point_added;
}

template<class T>
void PRM<T>::set_borders(std::vector<HeapNode<HeapPoint3D> *> borders) {
	this->rand_max_x = -std::numeric_limits<double>::max();
	this->rand_max_y = this->rand_max_x;
	this->rand_max_z = this->rand_max_x;
	this->rand_min_x = std::numeric_limits<double>::max();
	this->rand_min_y = this->rand_min_x;
	this->rand_min_z = this->rand_min_x;
	for (int var = 0; var < borders.size(); ++var) {
		this->rand_max_x = MAX(borders[var]->data.x, this->rand_max_x);
		this->rand_max_y = MAX(borders[var]->data.y, this->rand_max_y);
		this->rand_max_z = MAX(borders[var]->data.z, this->rand_max_z);
		this->rand_min_x = MIN(borders[var]->data.x, this->rand_min_x);
		this->rand_min_y = MIN(borders[var]->data.y, this->rand_min_y);
		this->rand_min_z = MIN(borders[var]->data.z, this->rand_min_z);
	}
}

template<class T>
void PRM<T>::calculate_added_points(bool add_points) {
	if (continuous_point_adding) {
		//INFO("calculate_added_points begin");
		//INFO("actually we have "<<generatedFreePositions.size()<<" points")
		//INFO("flann_index.size() " <<flann_index->size()<<" generatedFreePositions.size() "<<generatedFreePositions.size());
		//INFO(	"we added "<<(generatedFreePositions.size()-continuous_point_adding_start_id) <<" points to existing " << continuous_point_adding_start_id<<" points")
		CShape color_new_points("red", "red", 1, 1);
		//INFO("new_points bef");
		std::vector<HeapNode<T> *> new_points(generatedFreePositions.begin() + continuous_point_adding_start_id,
				generatedFreePositions.begin() + generatedFreePositions.size());
		//INFO("new_points af");
		//draw_points(new_points, &color_new_points,true);
		draw_points(generatedFreePositions);
		//connect new points to the map
		flann::Matrix<float> new_points_matrix;
		//INFO("want to add " <<new_points.size() <<" points to flann index");
		if (planning_state == state2d) {
			new_points_matrix = flann::Matrix<float>(new float[new_points.size() * 2], new_points.size(), 2);
			for (int var = 0; var < new_points.size(); ++var) {
				new_points_matrix[var][0] = new_points[var]->data.x;
				new_points_matrix[var][1] = new_points[var]->data.y;
				//INFO(*new_points[var]);
			}
			if (add_points) {
				flann_index->addPoints(new_points_matrix, 2.0);	//do not add points when initializing
			}
		} else if (planning_state == state2dheading) {
			new_points_matrix = flann::Matrix<float>(new float[new_points.size() * 3], new_points.size(), 3);
			for (int var = 0; var < new_points.size(); ++var) {
				new_points_matrix[var][0] = new_points[var]->data.x;
				new_points_matrix[var][1] = new_points[var]->data.y;
				new_points_matrix[var][2] = ((HeapNode<HeapPoint2DHeading>*) new_points[var])->data.phi;
			}
			if (add_points) {
				flann_index_dub->addPoints(new_points_matrix, 2.0);	//do not add points when initializing
			}
		} else if (planning_state == state3d) {
			new_points_matrix = flann::Matrix<float>(new float[new_points.size() * 3], new_points.size(), 3);
			for (int var = 0; var < new_points.size(); ++var) {
				new_points_matrix[var][0] = new_points[var]->data.x;
				new_points_matrix[var][1] = new_points[var]->data.y;
				new_points_matrix[var][2] = ((HeapNode<HeapPoint3D>*) new_points[var])->data.z;
				//INFO(*new_points[var]);
			}
			if (add_points) {
				flann_index->addPoints(new_points_matrix, 2.0);	//do not add points when initializing
			}
		}
		//INFO("added points");

		float sd = 2; //configuration space dimension
		if (planning_state == state2dheading) {
			sd = 3;
		}
		float num_points = generatedFreePositions.size();

		int k = M_E * (1 + 1 / sd) * log10(num_points);
		//k += num_headings;
		int k_search = k;
		if (planning_state == state2dheading) {
			k_search = k * 10;
		}
		//INFO_VAR(k);
		//INFO_VAR(k_search);
		std::vector<std::vector<int>> indices;
		std::vector<std::vector<int>> indices_rew;
		std::vector<std::vector<float>> dists;
		//flann::Matrix<int> indices(new int[new_points_matrix.rows * k_search], new_points_matrix.rows, k_search);
		//flann::Matrix<float> dists(new float[new_points_matrix.rows * k_search], new_points_matrix.rows, k_search);
		if (planning_state == state2d || planning_state == state3d) {
			flann_index->knnSearch(new_points_matrix, indices, dists, k_search, flann::SearchParams(128));
		} else if (planning_state == state2dheading) {
			flann_index_dub->knnSearch(new_points_matrix, indices, dists, k_search, flann::SearchParams(128));
			INFO("after knnSearch flann_index.size() " <<flann_index_dub->size());
		}

		indices_rew = indices;
		int numConnectionsAdded = 0;
		int numConnectionsAlreadyAdded = 0;
		int addedCounter = 0;
		int max_nn_used = 0;
		//INFO("add connections begin with k="<<k);
		//connect all points to generated_number_of_neighbors positions
		for (int generateIndex = continuous_point_adding_start_id; generateIndex < generatedFreePositions.size();
				++generateIndex) {
			HeapNode<T>* actual = generatedFreePositions[generateIndex];
			if (planning_state == state2dheading) {
				//precalculate the knn for dubins
				int max_nn_act = filter_nn(actual, indices, indices_rew, generateIndex, k, k_search);
				max_nn_used = MAX(max_nn_used, max_nn_act);
			}
			int connection_per_target = 0;

			for (int neigbourIndex = 0; neigbourIndex < k; ++neigbourIndex) {
				int nnindex = indices[generateIndex - continuous_point_adding_start_id][neigbourIndex];
				if (generatedFreePositions[nnindex]->cluster_id == actual->cluster_id) {
					//INFO("skip same id gi "<<generateIndex<<" ni "<<neigbourIndex<<" for cluster id "<< actual->cluster_id)
					continue;
				}

				HeapNode<T>* neigbour = generatedFreePositions[nnindex];

				bool freePath_act_neigh = isPathFreeBetweenNodes(actual, neigbour, obstacles, robot);
				if (freePath_act_neigh) {
					double distance_act_neigh = actual->data.distance(neigbour->data);
					//INFO("distance_act_neigh "<<distance_act_neigh<<" dists[generateIndex - continuous_point_adding_start_id][neigbourIndex] "<<sqrt(dists[generateIndex - continuous_point_adding_start_id][neigbourIndex]))

					auto connectionAdded1 = actual->visibility_node_ids.insert(
							std::pair<HeapNode<T>*, double>(neigbour, distance_act_neigh)); //addConnection(neigbour, bestDist[neigbourIndex]);

					if (connectionAdded1.second) {
						numConnectionsAdded++;
						addedCounter++;
						connection_per_target += 1;
					} else {
						numConnectionsAlreadyAdded++;
					}
				}

			}

			for (int neigbourIndex = 0; neigbourIndex < k; ++neigbourIndex) {
				int nnindex = indices_rew[generateIndex - continuous_point_adding_start_id][neigbourIndex];
				if (generatedFreePositions[nnindex]->cluster_id == actual->cluster_id) {
					//INFO("skip same id gi "<<generateIndex<<" ni "<<neigbourIndex<<" for cluster id "<< actual->cluster_id)
					continue;
				}

				HeapNode<T>* neigbour = generatedFreePositions[nnindex];
				bool freePath_neigh_act = isPathFreeBetweenNodes(neigbour, actual, obstacles, robot);
				if (freePath_neigh_act) {
					double distance_neigh_act = neigbour->data.distance(actual->data);
					auto connectionAdded2 = neigbour->visibility_node_ids.insert(
							std::pair<HeapNode<T>*, double>(actual, distance_neigh_act));
					if (connectionAdded2.second) {
						numConnectionsAdded++;
						addedCounter++;
						connection_per_target += 1;
					} else {
						numConnectionsAlreadyAdded++;
					}
				}
			}
			//INFO("snode "<<actual->node_id<<" gen index "<<generateIndex<<" c added "<<connection_per_target<< " k is "<<k)

		}

		//INFO_RED("max_nn_used "<<max_nn_used);
		this->continuous_point_adding_start_id = generatedFreePositions.size();
		this->continuous_point_adding = false;
		//INFO("calculate_added_points end");
	} else {
		INFO("no new points");
	}
}

template<class T>
plan_with_length<T> PRM<T>::plan(int start_index, int goal_index) {
//INFO("begin PRM::plan()");

//INFO("dijskra.findPath ");
	std::vector<int> goal_indexes;
	goal_indexes.push_back(goal_index);
	std::vector<plan_with_length<T>> found_paths = dijskra.findPath(start_index, goal_indexes, generatedFreePositions);

//INFO("plan length "<<found_paths[0].lenght);
//INFO("end PRM::plan()");

	return found_paths[0];
}

template<class T>
std::vector<plan_with_length<T>> PRM<T>::plan(int start_index, std::vector<int> goal_indexes) {
	//INFO("begin PRM::plan()");

	std::vector<plan_with_length<T>> found_paths = dijskra.findPath(start_index, goal_indexes, generatedFreePositions);

	//INFO("end PRM::plan()");
	return found_paths;
}

template<class T>
std::vector<HeapNode<T>*>& PRM<T>::get_points() {
	return this->generatedFreePositions;
}

template<class T>
bool PRM<T>::testCollision(std::vector<MeshObject*> obstacles, MeshObject* object, Position3D object_position) {
	return MeshObject::collide(&obstacles, object, object_position);
}

template<class T>
void PRM<T>::set_gui(CCanvasBase* canvas_) {
	this->canvas = canvas_;
}

template<class T>
void PRM<T>::set_num_headings(int num_headings) {
	this->num_headings = num_headings;
}

template<class T>
void PRM<T>::draw_points(std::vector<HeapNode<T> *> &points, CShape* color, bool clear_actual) {
	if (canvas) {
		//INFO("draw points begin")
		CShape neighborhoodPoint("blue", "blue", 1, 1);
		if (color != NULL) {
			neighborhoodPoint = *color;
		}
		if (clear_actual) {
			*canvas << canvas::CLEAR << "prm_points";
		}
		*canvas << "prm_points" << canvas::POINT;
		for (int var = 0; var < points.size(); ++var) {
			*canvas << neighborhoodPoint << points[var]->data.x << points[var]->data.y;
		}
		*canvas << canvas::END;
		canvas->redraw();
		//INFO("draw points end")
	}
}

template<class T>
void PRM<T>::draw_connections(std::vector<HeapNode<T> *> &points, CShape* color, bool clear_actual) {
	if (canvas) {
		CShape neighborhoodPoint("green", "green", 1, 1);
		if (color != NULL) {
			neighborhoodPoint = *color;
		}
		if (clear_actual) {
			*canvas << canvas::CLEAR << "prm_connections";
		}
		*canvas << "prm_connections" << canvas::LINE;
		for (int var = 0; var < points.size(); ++var) {
			for (auto connection : points[var]->visibility_node_ids) {
				*canvas << neighborhoodPoint << points[var]->data.x << points[var]->data.y << connection.first->data.x
						<< connection.first->data.y;
			}

		}
		*canvas << canvas::END;
		canvas->redraw();
	}
}

#endif /* SRC_PLANNER_PRM_H_ */
