/*
 * RRT.h
 *
 *  Created on: 16. 11. 2014
 *      Author: robert
 */

#ifndef RRT_H_
#define RRT_H_
#include "crl/logging.h"
#include "my_defines.h"
#include <vector>
#include "math_common.h"
#include "heuristic_types.h"
#include <algorithm>
#include <crl/config.h>
#include "my_defines.h"
#include <limits>
#include "heap.h"
#include "dijkstra.h"

#include "opendubins/dubins.h"
#include <flann/flann.hpp>

//#include "lib/mpnn/MPNN/include/DNN/ANN.h"
//#include "lib/mpnn/MPNN/include/DNN/multiann.h"
#include "mesh_object.h"

#include <crl/alg/algorithm.h>
#include "crl/gui/canvas.h"
#include <crl/gui/shape.h>

#define OUTPUT_AFTER_ADDED_NUM_RRT 100
#define MAX_NUM_VERTICES_RRT 5000000

/* UNCOMMENT FOR BAUNDING BOX GENERATING */
/*
 #define GENERATE_RANDOM_IN_BOUND_BOX			//if to generate new random only in min-max cube
 #define RANGE_PART_ALLOWED (0.3)
 */
template<typename T>
struct TreeNode;

template<typename T>
struct TreeNode {
	int node_id;
	int cluster_id;
	T data;
	std::vector<TreeNode<T> *> children;
	TreeNode<T> * parent;
	double distance_from_start;
//static int nodeCount;
//static std::vector<TreeNode<T> *> nodes;
	int vector_position; //bool visited;
	bool city_node;

	void setParent(TreeNode<T> * parent_){
		this->parent = parent_;
	}
	TreeNode<T> * getParent(){
		return this->parent;
	}
	void addChild(TreeNode<T> * new_child_){
		children.push_back(new_child_);
	}
};


template<typename T>
struct tree_plan_with_length {
	std::vector<TreeNode<T>*> plan;
	double lenght;
	int from_id;
	int to_id;
};

using namespace crl::gui;
using namespace opendubins;
using crl::logger;

template<class T>
class RRT {
public:
	RRT(crl::CConfig& config, PlanningState planning_state_);
	virtual ~RRT();
	void initialize(std::vector<MeshObject*> obstacles_, MeshObject* robot_, std::vector<HeapNode<T>*> &cities_nodes_,
			int start_index_);
	//plan_with_length<T> plan(int start_index, int goal_index, std::vector<MeshObject*> obstacles, MeshObject* robot);
	tree_plan_with_length<T> plan(int start_index, int goal_index);
	void set_borders(std::vector<HeapNode<HeapPoint2D> *> borders);
	void set_gui(CCanvasBase* canvas_);
	void draw_points(std::vector<TreeNode<T> *> &points, CShape* color = NULL, bool clear_actual = NULL);
	void draw_connections(std::vector<TreeNode<T> *> &points, CShape* color = NULL, bool clear_actual = NULL);
	std::vector<TreeNode<T> *> get_tree_nodes();

	//void expand_in_direction(GraphNode<T> random_city);
private:
	//used only when using gui

	bool fillRandomState(TreeNode<T> * positionToFill);
	//bool fillRandomState(AUV_State * positionToFill);

	// GET POINT FROM NEAREST NEIGHBOR NODE AND RANDOM NODE
	double getPointFromNN(TreeNode<T> * newNodeRandom, TreeNode<T> * nearestNeighbor, double distance);
	//void getPointFromNN(TreeNode<AUV_State> * newNodeRandom,TreeNode<AUV_State> * nearestNeighbor,double distance);

	//test collision
	bool hasCollision(std::vector<MeshObject*> obstacles, MeshObject* object, TreeNode<T>* node);
	//bool testCollision(std::vector<MeshObject*> obstacles, MeshObject* object,AUV_State objectState);

	//TreeNode<T> * findNearestNeighbor(MPNN::ANNpoint startPoint, double* bestDistance);
	TreeNode<T> * findNearestNeighbor(TreeNode<T> * testing_point, double* bestDistance,
			std::vector<std::vector<int>> & indices, std::vector<std::vector<int>> & indices_rew,
			std::vector<std::vector<float>> & dists, flann::Matrix<float> & new_points_matrix);

	void fillPlan(tree_plan_with_length<T>* foundPlan);

	//FILL OR CREATE ANN POINT
	//MPNN::ANNpoint fillANNpoint(TreeNode<Position3D> * treeNode, MPNN::ANNpoint aNNpoint = NULL);
	//MPNN::ANNpoint fillANNpoint(TreeNode<AUV_State> * treeNode , MPNN::ANNpoint aNNpoint = NULL );

	//double calcDistance(TreeNode<Position3D> * newNodeRandom, TreeNode<Position3D> * nearestNeighbor);

	// start anf goal states
	TreeNode<T> * start;
	TreeNode<T> * goal;
	PlanningState planning_state;
	//kdtree variables

	CCanvasBase* canvas;

	flann::Index<flann::L2<float>> *flann_index;

	double dubins_radius;
	flann::Index<flann::L2Dubins<float>> *flann_index_dub;

	std::vector<MeshObject*> obstacles;
	MeshObject* robot;
	std::vector<HeapNode<T> *> cities_nodes;
	int start_index;
	int goal_index;

	std::vector<TreeNode<T> *> generatedNodes;

	double collision_distance_check;

	double rand_min_x;
	double rand_max_x;
	double rand_min_y;
	double rand_max_y;
	double rand_min_z;
	double rand_max_z;
};

template<class T>
RRT<T>::RRT(crl::CConfig& config, PlanningState planning_state_) {
	this->planning_state = planning_state_;
	this->dubins_radius = config.get<double>("dubins-radius");
	this->collision_distance_check = config.get<double>("collision-distance-check");
}

template<class T>
RRT<T>::~RRT() {

}

template<class T>
void RRT<T>::initialize(std::vector<MeshObject*> obstacles_, MeshObject* robot_,
		std::vector<HeapNode<T>*> &cities_nodes_, int start_index_) {
	this->cities_nodes = cities_nodes_;
	this->robot = robot_;
	this->obstacles = obstacles_;
	this->start_index = start_index_;
}

template<class T>
tree_plan_with_length<T> RRT<T>::plan(int start_index, int goal_index) {
	INFO("begin RRT::plan()");
	INFO_VAR(collision_distance_check);
	INFO_VAR(start_index);
	INFO_VAR(goal_index);
	tree_plan_with_length<T> foundPlan;

	this->start = new TreeNode<T>();
	this->start->data = this->cities_nodes[start_index]->data;
	this->start->city_node = this->cities_nodes[start_index]->city_node;
	this->start->cluster_id = this->cities_nodes[start_index]->cluster_id;
	this->start->node_id = this->cities_nodes[start_index]->node_id;
	this->start->vector_position = 0;
	this->start->setParent(NULL);
	this->start->distance_from_start = 0;

	INFO("start TreeNode " << this->start);
	INFO("start is at "<<this->start->data.x<<" "<<this->start->data.y)
	generatedNodes.push_back(this->start);

	this->goal = new TreeNode<T>();
	this->goal->data = this->cities_nodes[goal_index]->data;
	this->goal->setParent(NULL);
	this->goal->distance_from_start = NAN;
	this->goal->city_node = this->cities_nodes[goal_index]->city_node;
	this->goal->cluster_id = this->cities_nodes[goal_index]->cluster_id;
	this->goal->node_id = this->cities_nodes[goal_index]->node_id;

	INFO("goal TreeNode " << this->goal);
	INFO("goal is at "<<this->goal->data.x<<" "<<this->goal->data.y)

	//add start point to KD tree
	INFO("create Flann ");

	if (planning_state == state2d) {
		flann::Matrix<float> start_matrix(new float[1 * 2], 1, 2);
		start_matrix[0][0] = this->start->data.x;
		start_matrix[0][1] = this->start->data.y;
		flann_index = new flann::Index<flann::L2<float>>(start_matrix, flann::KDTreeIndexParams(4));
		flann_index->buildIndex();

	} else if (planning_state == state2dheading) {
		flann::Matrix<float> start_matrix(new float[1 * 3], 1, 3);
		start_matrix[0][0] = this->start->data.x;
		start_matrix[0][1] = this->start->data.y;
		start_matrix[0][2] = ((HeapNode<HeapPoint2DHeading>*) this->start)->data.phi;
		flann::L2Dubins<float>::radius = this->dubins_radius;
		flann_index_dub = new flann::Index<flann::L2Dubins<float>>(start_matrix, flann::KDTreeIndexParams(4));
		flann_index_dub->buildIndex();
	}

	//create ann point that is then filled all the time for NN
	//MPNN::ANNpoint newNodeANN = fillANNpoint(this->start);
	int countAdded = 0;
	int collisionCounter = 0;
	int var;
	//std::cout << "distances=[" << std::endl;
	bool collision = true;
	bool planFound = false;
	bool newAdded = false;

	std::vector<std::vector<int>> indices;
	std::vector<std::vector<int>> indices_rew;
	std::vector<std::vector<float>> dists;
	flann::Matrix<float> search_matrix;
	if (planning_state == state2d) {
		search_matrix = flann::Matrix<float>(new float[1 * 2], 1, 2);
	} else {
		search_matrix = flann::Matrix<float>(new float[1 * 3], 1, 3);
	}
	//Position3D* newNodeRandomPosition = new Position3D();
	TreeNode<T> * tempRandomNode = new TreeNode<T>();
	INFO("before adding rrt nodes ");
	for (var = 0; var < MAX_NUM_VERTICES_RRT; ++var) {
		//std::cout << var << std::endl;

		TreeNode<T> * nearestNeighbor=NULL;
		double distance;
		double rotationMaxChanger;
		bool goalUsed = false;
		int part = 0;
		newAdded = false;
		INFO( "bef while " );
		while (!newAdded) {
			TreeNode<T>* newNode = new TreeNode<T>();
			//INFO("generated "<<generateIndex);
			newNode->node_id = generatedNodes.size();	//start and goal are 0 and 1
			newNode->cluster_id = newNode->node_id;	//start and goal are 0 and 1
			newNode->city_node = false;

			goalUsed = fillRandomState(tempRandomNode);
			if (goalUsed) {
				INFO("goal used to fill random state");
			}
			nearestNeighbor = findNearestNeighbor(tempRandomNode, &distance, indices, indices_rew, dists,
					search_matrix);


			INFO("found nearest neighbor "<<nearestNeighbor<<" in dist "<<distance);
			//new nearest neighbor
			//nearestNeighbor = findNearestNeighbor(newNodeANN, &distance);

			distance = getPointFromNN(tempRandomNode, nearestNeighbor, distance);
			INFO("point in distance "<<distance<<" from NN is "<<tempRandomNode->data.x<<" "<<tempRandomNode->data.y);
			INFO("test collision");
			if (!hasCollision(obstacles, robot, tempRandomNode)) {
				//std::cout << newNode->position << std::endl;
				INFO("is collision free");
				newNode->data = tempRandomNode->data;

				//INFO(newNode << " has NN " << nearestNeighbor);

				//kdTree->AddPoint(newNodeANN, newNode);
				if (planning_state == state2d) {
					flann::Matrix<float> add_point_matrix(new float[1 * 2], 1, 2);
					add_point_matrix[0][0] = newNode->data.x;
					add_point_matrix[0][1] = newNode->data.y;
					flann_index->addPoints(add_point_matrix, 2.0);
				} else if (planning_state == state2dheading) {
					flann::Matrix<float> add_point_matrix(new float[1 * 3], 1, 3);
					add_point_matrix[0][0] = newNode->data.x;
					add_point_matrix[0][1] = newNode->data.y;
					add_point_matrix[0][2] = ((HeapNode<HeapPoint2DHeading>*) newNode)->data.phi;
					flann_index_dub->addPoints(add_point_matrix, 2.0);
				}

				nearestNeighbor->addChild(newNode);
				newNode->setParent(nearestNeighbor);
				newNode->distance_from_start = nearestNeighbor->distance_from_start + distance;
				INFO("add newNode "<<newNode);
				INFO("with pos "<<newNode->data.x<<" "<<newNode->data.y);
				generatedNodes.push_back(newNode);
				countAdded++;

				if (countAdded >= OUTPUT_AFTER_ADDED_NUM_RRT) {
					//INFO("collision states "<<(100.0*((double)collisionCounter)/((double) (collisionCounter+countAdded)))<<"%");
					countAdded = 0;
					collisionCounter = 0;
					INFO(nearestNeighbor->node_id << " add child " << newNode->node_id);

					/*
					 if (gui != NULL) {
					 Gui::showTree(this->start);
					 }
					 */
				}

				collision = false;
				newAdded = true;
				//std::cout << "newAdded" << std::endl;
			} else {
				collision = true;
				collisionCounter++;
			}
		}

		//after successfull added node test if plan found
		nearestNeighbor = findNearestNeighbor(goal, &distance, indices, indices_rew, dists, search_matrix);
		INFO("nearestNeighbor to goal is "<<nearestNeighbor<< " in distance "<<distance);
		INFO("goal pos "<<goal->data.x<<" "<<goal->data.y);
		INFO("nn   pos "<<nearestNeighbor->data.x<<" "<<nearestNeighbor->data.y);
		//findNearestNeighbor(goalPoint, &distance);
		if (distance < collision_distance_check) {
			planFound = true;
			goal->setParent(nearestNeighbor);
			goal->distance_from_start = goal->parent->distance_from_start + distance;
			INFO("found goal with " << (var + 1) << "nodes");
			break;
		}
	}
	INFO("];");
	fillPlan(&foundPlan);
	INFO("end RRT::plan()");
	return foundPlan;
}

template<class T>
TreeNode<T> * RRT<T>::findNearestNeighbor(TreeNode<T> * testing_point, double* bestDistance,
		std::vector<std::vector<int>> & indices, std::vector<std::vector<int>> & indices_rew,
		std::vector<std::vector<float>> & dists, flann::Matrix<float> & new_points_matrix) {

	if (planning_state == state2d) {
		new_points_matrix[0][0] = testing_point->data.x;
		new_points_matrix[0][1] = testing_point->data.y;
		flann_index->knnSearch(new_points_matrix, indices, dists, 1, flann::SearchParams(128));
	} else if (planning_state == state2dheading) {
		new_points_matrix[0][0] = testing_point->data.x;
		new_points_matrix[0][1] = testing_point->data.y;
		new_points_matrix[0][2] = ((HeapNode<HeapPoint2DHeading>*) testing_point)->data.phi;
		flann_index_dub->knnSearch(new_points_matrix, indices, dists, 1, flann::SearchParams(128));
	}

	int nnindex = indices[0][0];
	*bestDistance = sqrt(dists[0][0]);
	return generatedNodes[nnindex];
}

template<class T>
void RRT<T>::set_borders(std::vector<HeapNode<HeapPoint2D> *> borders) {
	this->rand_max_x = -std::numeric_limits<double>::max();
	this->rand_max_y = this->rand_max_x;
	this->rand_max_z = 0;
	this->rand_min_x = std::numeric_limits<double>::max();
	this->rand_min_y = this->rand_min_x;
	this->rand_min_z = 0;
	for (int var = 0; var < borders.size(); ++var) {
		this->rand_max_x = MAX(borders[var]->data.x, this->rand_max_x);
		this->rand_max_y = MAX(borders[var]->data.y, this->rand_max_y);
		this->rand_min_x = MIN(borders[var]->data.x, this->rand_min_x);
		this->rand_min_y = MIN(borders[var]->data.y, this->rand_min_y);
	}
}

template<class T>
void RRT<T>::set_gui(CCanvasBase* canvas_) {
	this->canvas = canvas_;
}

template<class T>
void RRT<T>::draw_points(std::vector<TreeNode<T> *> &points, CShape* color, bool clear_actual) {
	if (canvas) {
		INFO("draw points begin")
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
		INFO("draw points end")
	}
}

template<class T>
void RRT<T>::draw_connections(std::vector<TreeNode<T> *> &points, CShape* color, bool clear_actual) {
	if (canvas) {
		INFO("draw connections");
		CShape neighborhoodPoint("green", "green", 2, 2);
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

template<class T>
void RRT<T>::fillPlan(tree_plan_with_length<T>* foundPlan) {
	TreeNode<T> *node = this->goal;
	foundPlan->lenght = this->goal->distance_from_start;
	foundPlan->from_id = this->start_index;
	foundPlan->to_id = this->goal_index;

	while (node != NULL) {
		//std::cout << node->id << std::endl;
		foundPlan->plan.push_back(node);
		node = node->getParent();
	}
	std::reverse(foundPlan->plan.begin(), foundPlan->plan.end());
}

template<class T>
std::vector<TreeNode<T> *> RRT<T>::get_tree_nodes() {
	return this->generatedNodes;
}

#endif /* RRT_H_ */
