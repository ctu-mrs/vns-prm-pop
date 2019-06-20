/*
 * Dijkstra.h
 *
 *  Created on: 2. 2. 2015
 *      Author: Robert Pěnička
 */

#ifndef SRC_PLANNER_DIJKSTRA_H_
#define SRC_PLANNER_DIJKSTRA_H_

#include <vector>
#include "float.h"
#include "heap.h"
#include <algorithm>
#include "my_defines.h"

#define DIJKSTRA_INF DBL_MAX

template<typename T>
struct plan_with_length {
	std::vector<HeapNode<T>> plan;
	double lenght;
	int from_id;
	int to_id;
};

template <class T>
class Dijkstra {
public:
	Dijkstra();
	virtual ~Dijkstra();
	//plan_with_length findPath(int start_index, int goal_index,		std::vector<HeapNode*> & connectedPoints);
	std::vector<plan_with_length<T>> findPath(int start_index, std::vector<int> goal_indexes,
			std::vector<HeapNode<T>*> & connectedPoints);

	private:
	HeapNode<T>* expandBestNode();
	Heap<HeapNode<T>*> * heap;
};

template <class T>
std::ostream& operator <<(std::ostream &o, const HeapNode<T>& p);


template<typename T>
Dijkstra<T>::Dijkstra() {
	heap = NULL;
}

template<typename T>
Dijkstra<T>::~Dijkstra() {
	if ( heap != NULL ) {
		delete heap;
	}
}

template<typename T>
std::vector<plan_with_length<T>> Dijkstra<T>::findPath(int start_index, std::vector<int> goal_indexes,
		std::vector<HeapNode<T>*> & visibility_graph) {
	//INFO("begin Dijkstra::findPath()");
	/*********** init dijkstra *************/


	for ( int var = 0 ; var < visibility_graph.size() ; ++var ) {
		visibility_graph[var]->distance_from_start = DIJKSTRA_INF;
		visibility_graph[var]->previous_point = NULL;
	}
	visibility_graph[start_index]->distance_from_start = 0;
	visibility_graph[start_index]->previous_point = visibility_graph[start_index];


	//need to create heap after setting points
	heap = new Heap<HeapNode<T>*>(visibility_graph);
	bool stopped = false;
	HeapNode<T>* actualBestNode;
	std::set<int> goal_indexes_non_visited(goal_indexes.begin(), goal_indexes.end());
	int numLooped = 0;
	//int loopCounter = 0;
	//INFO("loop points");
	while ( !stopped ) {

		actualBestNode = expandBestNode();

		if ( actualBestNode == NULL || actualBestNode->distance_from_start == DIJKSTRA_INF ) {
			stopped = true;
		}

		if ( !stopped && actualBestNode->city_node ) {
			//can be one of goal
			auto search = goal_indexes_non_visited.find(actualBestNode->node_id);
			if ( search != goal_indexes_non_visited.end() ) {
				goal_indexes_non_visited.erase(search);
			}

			if ( goal_indexes_non_visited.empty() ) {
				stopped = true;
			}
		}

		numLooped++;
		/*
		loopCounter++;
		if ( loopCounter >= 1000 ) {
			loopCounter = 0;
			//INFO("looped " << numLooped << " points");
		}
		*/
	}

	delete heap;
	heap = NULL;
	//INFO("plan");

	std::vector<plan_with_length<T>> plans_w_lenght;

	//INFO("createPlan");
	//INFO("goal " << goal->node_id << " previous has "<< goal->previous_point->node_id);
	for ( int var = 0 ; var < goal_indexes.size() ; ++var ) {

		std::vector<HeapNode<T>> plan;
		HeapNode<T>* actualPoint = visibility_graph[goal_indexes[var]];
		//INFO("createPlan from goal " << actualPoint<< " node id "<<actualPoint->node_id <<" is city "<<actualPoint->city_node);
		if ( actualPoint->previous_point != NULL ) {
			plan.push_back(*actualPoint);
			while ( actualPoint != visibility_graph[start_index] ) {
				//WINFO(actualPoint << " get previous point ");
				actualPoint = actualPoint->previous_point;
				//INFO("add point " << actualPoint<< " node id "<<actualPoint->node_id <<" is city "<<actualPoint->city_node);
				plan.push_back(*actualPoint);
			}
			std::reverse(plan.begin(), plan.end());
			plan_with_length<T> plan_w_lenght;
			plan_w_lenght.lenght = visibility_graph[goal_indexes[var]]->distance_from_start;
			plan_w_lenght.plan = plan;
			plan_w_lenght.from_id = start_index;
			plan_w_lenght.to_id = goal_indexes[var];
			plans_w_lenght.push_back(plan_w_lenght);
		} else {
			plan_with_length<T> plan_w_lenght;
			plan_w_lenght.lenght = DIJKSTRA_INF;
			plan_w_lenght.plan = plan;
			plan_w_lenght.from_id = start_index;
			plan_w_lenght.to_id = goal_indexes[var];
			plans_w_lenght.push_back(plan_w_lenght);
		}

	}
	//INFO("end Dijkstra::findPath()");

	return plans_w_lenght;
}

template<typename T>
HeapNode<T>* Dijkstra<T>::expandBestNode() {
	HeapNode<T>* expandingNode = heap->pop();
	if ( expandingNode != NULL ) {
		//INFO("expandingNode " << expandingNode->node_id << " with " << expandingNode->distance_from_start << " is city " << expandingNode->city_node);
		//WINFO("expandingNode has " << expandingNode->getConnections().size()<<" connected points");
		for ( auto connectedNode : expandingNode->visibility_node_ids ) {
			//for ( int var = 0 ; var < expandingNode->visibility_node_ids.size() ; ++var ) {
			//HeapNode* connectedNode = expandingNode->visibility_node_ids[var];
			double calculated_new_distance = expandingNode->distance_from_start + connectedNode.second;

			if ( calculated_new_distance < connectedNode.first->distance_from_start ) {
				//INFO_RED("redo calculated_additional_distance");
				//test point if better distance found
				connectedNode.first->previous_point = expandingNode;
				heap->updateCost(connectedNode.first, calculated_new_distance);
				//INFO("\tconnectedNode " << connectedNode->node_id << " with " << connectedNode->distance_from_start << " is city " << expandingNode->city_node);
			}
		}
	}
	return expandingNode;
}

#endif /* SRC_PLANNER_DIJKSTRA_H_ */
