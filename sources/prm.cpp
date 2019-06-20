/*
 * PRM.cpp
 *
 *  Created on: 1. 2. 2015
 *      Author: Robert Pěnička
 */

#include "prm.h"

template<>
void PRM<HeapPoint2D>::fillRandomState(HeapNode<HeapPoint2D> * positionToFill) {
	positionToFill->city_node = false;
	positionToFill->data.x = randDoubleMinMax(rand_min_x, rand_max_x);
	positionToFill->data.y = randDoubleMinMax(rand_min_y, rand_max_y);
	//INFO("fill random state done")
}

template<>
void PRM<HeapPoint2DHeading>::fillRandomState(HeapNode<HeapPoint2DHeading> * positionToFill) {
	positionToFill->city_node = false;
	positionToFill->data.x = randDoubleMinMax(rand_min_x, rand_max_x);
	positionToFill->data.y = randDoubleMinMax(rand_min_y, rand_max_y);
	positionToFill->data.phi = randDoubleMinMax(0, M_2PI);
	positionToFill->data.radius = this->dubins_radius;
	//INFO("fill random state HeapPoint2DHeading done")
}

template<>
void PRM<HeapPoint3D>::fillRandomState(HeapNode<HeapPoint3D> * positionToFill) {
	positionToFill->city_node = false;
	positionToFill->data.x = randDoubleMinMax(rand_min_x, rand_max_x);
	positionToFill->data.y = randDoubleMinMax(rand_min_y, rand_max_y);
	positionToFill->data.z = randDoubleMinMax(rand_min_z, rand_max_z);
	//INFO("fill random state HeapPoint2DHeading done")
}

template<>
void PRM<HeapPoint2D>::fillPoint(HeapNode<HeapPoint2D>* positionToFill, Point3DOriented point) {
	positionToFill->data.x = point.x;
	positionToFill->data.y = point.y;
}

template<>
void PRM<HeapPoint2DHeading>::fillPoint(HeapNode<HeapPoint2DHeading>* positionToFill, Point3DOriented point) {
	positionToFill->data.x = point.x;
	positionToFill->data.y = point.y;
	positionToFill->data.phi = point.yaw;
	positionToFill->data.radius = this->dubins_radius;
}

template<>
void PRM<HeapPoint3D>::fillPoint(HeapNode<HeapPoint3D>* positionToFill, Point3DOriented point) {
	positionToFill->data.x = point.x;
	positionToFill->data.y = point.y;
	positionToFill->data.z = point.z;
}

template<>
bool PRM<HeapPoint2D>::isPathFreeBetweenNodes(HeapNode<HeapPoint2D>* actual, HeapNode<HeapPoint2D>* neigbour,
		std::vector<MeshObject*> & obstacles, MeshObject* robot) {
	double freeBetween = true;
	double distanceToNeigbor = actual->data.distance(neigbour->data);
	double distanceMaxChanger = distanceToNeigbor / collision_distance_check;
	double t, t1, distanceAfter;
	Position3D object_position;
	object_position.x = 0;
	object_position.y = 0;
	object_position.z = 0;
	object_position.yaw = 0;
	object_position.pitch = 0;
	object_position.roll = 0;
	HeapNode<HeapPoint2D> position_between;
	double index = 1.0;
	for ( ; index * collision_distance_check < distanceToNeigbor ; index += 1.0 ) {
		object_position.x = (neigbour->data.x + index * (actual->data.x - neigbour->data.x) / distanceMaxChanger);
		object_position.y = (neigbour->data.y + index * (actual->data.y - neigbour->data.y) / distanceMaxChanger);
		if ( testCollision(obstacles, robot, object_position) ) {
			freeBetween = false;
			break;
		}
	}
	return freeBetween;
}

template<>
bool PRM<HeapPoint2DHeading>::isPathFreeBetweenNodes(HeapNode<HeapPoint2DHeading>* actual, HeapNode<HeapPoint2DHeading>* neigbour,
		std::vector<MeshObject*>& obstacles, MeshObject* robot) {
	double freeBetween = true;
	double t, t1, distanceAfter;
	//HeapNode<HeapPoint2DHeading> position_between;

	State start_state(actual->data.x, actual->data.y, actual->data.phi);
	State stop_state(neigbour->data.x, neigbour->data.y, neigbour->data.phi);
	double radius = actual->data.radius;
	Dubins dub(start_state, stop_state, radius);
	double dub_len = dub.length;
	Position3D object_position;
	object_position.x = 0;
	object_position.y = 0;
	object_position.z = 0;
	object_position.yaw = 0;
	object_position.pitch = 0;
	object_position.roll = 0;
	double test_len = collision_distance_check;
	for ( ; test_len < dub_len ; test_len += collision_distance_check ) {
		//INFO("test_len "<<test_len<<" of "<<dub_len);
		State state_between = dub.getState(test_len);
		object_position.x = state_between.point.x;
		object_position.y = state_between.point.y;
		object_position.yaw = state_between.ang;
		if ( testCollision(obstacles, robot, object_position) ) {
			freeBetween = false;
			break;
		}
	}
	return freeBetween;
}

template<>
bool PRM<HeapPoint3D>::isPathFreeBetweenNodes(HeapNode<HeapPoint3D>* actual, HeapNode<HeapPoint3D>* neigbour,
		std::vector<MeshObject*> & obstacles, MeshObject* robot) {
	double freeBetween = true;
	double distanceToNeigbor = actual->data.distance(neigbour->data);
	double distanceMaxChanger = distanceToNeigbor / collision_distance_check;
	double t, t1, distanceAfter;
	Position3D object_position;
	object_position.x = 0;
	object_position.y = 0;
	object_position.z = 0;
	object_position.yaw = 0;
	object_position.pitch = 0;
	object_position.roll = 0;
	HeapNode<HeapPoint3D> position_between;
	double index = 1.0;
	for ( ; index * collision_distance_check < distanceToNeigbor ; index += 1.0 ) {
		object_position.x = (neigbour->data.x + index * (actual->data.x - neigbour->data.x) / distanceMaxChanger);
		object_position.y = (neigbour->data.y + index * (actual->data.y - neigbour->data.y) / distanceMaxChanger);
		object_position.z = (neigbour->data.z + index * (actual->data.z - neigbour->data.z) / distanceMaxChanger);
		if ( testCollision(obstacles, robot, object_position) ) {
			freeBetween = false;
			break;
		}
	}
	return freeBetween;
}

template<>
bool PRM<HeapPoint2D>::testCollision(std::vector<MeshObject*> obstacles, MeshObject* object, HeapNode<HeapPoint2D>* node) {
	//INFO("testCollision HeapPoint2D");
	Position3D object_position;
	object_position.x = node->data.x;
	object_position.y = node->data.y;
	object_position.z = 0;
	object_position.yaw = 0;
	object_position.pitch = 0;
	object_position.roll = 0;
	return MeshObject::collide(&obstacles, object, object_position);
}

template<>
bool PRM<HeapPoint2DHeading>::testCollision(std::vector<MeshObject*> obstacles, MeshObject* object, HeapNode<HeapPoint2DHeading>* node) {
	Position3D object_position;
	object_position.x = node->data.x;
	object_position.y = node->data.y;
	object_position.z = 0;
	object_position.yaw = node->data.phi;
	object_position.pitch = 0;
	object_position.roll = 0;
	return MeshObject::collide(&obstacles, object, object_position);
}

template<>
bool PRM<HeapPoint3D>::testCollision(std::vector<MeshObject*> obstacles, MeshObject* object, HeapNode<HeapPoint3D>* node) {
	//INFO("testCollision HeapPoint2D");
	Position3D object_position;
	object_position.x = node->data.x;
	object_position.y = node->data.y;
	object_position.z = node->data.z;
	object_position.yaw = 0;
	object_position.pitch = 0;
	object_position.roll = 0;
	return MeshObject::collide(&obstacles, object, object_position);
}

template<>
int PRM<HeapPoint2D>::filter_nn(HeapNode<HeapPoint2D>* actual, std::vector<std::vector<int>> & indices, std::vector<std::vector<int>>& indices_rew,
		int generateIndex, int k, int k_search) {

}

template<>
int PRM<HeapPoint2DHeading>::filter_nn(HeapNode<HeapPoint2DHeading>* actual, std::vector<std::vector<int>> & indices,
		std::vector<std::vector<int>> & indices_rew, int generateIndex, int k, int k_search) {
	//INFO("filter from "<<indices[generateIndex - continuous_point_adding_start_id].size());
	std::map<double, std::pair<int,int>> distance_map;
	std::map<double, std::pair<int,int>> distance_map_rew;
	for ( int neigbourIndex = 0 ; neigbourIndex < k_search ; ++neigbourIndex ) {
		int nnindex = indices[generateIndex - continuous_point_adding_start_id][neigbourIndex];
		if ( generatedFreePositions[nnindex]->cluster_id == actual->cluster_id ) {
			continue;
		}
		HeapNode<HeapPoint2DHeading>* neigbour = generatedFreePositions[nnindex];
		State start_state(actual->data.x, actual->data.y, actual->data.phi);
		State stop_state(neigbour->data.x, neigbour->data.y, neigbour->data.phi);
		double radius = actual->data.radius;
		Dubins dub(start_state, stop_state, radius);
		//INFO(neigbourIndex<<" d="<<dub.length);
		distance_map[dub.length] = std::pair<int,int>(nnindex,neigbourIndex);
		if ( distance_map.size() > k ) {
			distance_map.erase(distance_map.rbegin()->first);
		}

		Dubins dub_rew(stop_state, start_state, radius);
		distance_map_rew[dub_rew.length] =  std::pair<int,int>(nnindex,neigbourIndex);
		if ( distance_map_rew.size() > k ) {
			distance_map_rew.erase(distance_map_rew.rbegin()->first);
		}

	}
	std::map<double, std::pair<int,int>>::iterator it;
	int i = 0;
	int max_used_nn = 0;
	//INFO("distances:");
	for ( it = distance_map.begin(); it != distance_map.end() ; it++ ) {
		//INFO(i<<" d="<<it->first);
		max_used_nn = MAX(max_used_nn, it->second.second);
		indices[generateIndex - continuous_point_adding_start_id][i] = it->second.first;
		i++;
	}
	i = 0;
	for ( it = distance_map_rew.begin(); it != distance_map_rew.end() ; it++ ) {
		//INFO(i<<" d="<<it->first);
		max_used_nn = MAX(max_used_nn, it->second.second);
		indices_rew[generateIndex - continuous_point_adding_start_id][i] = it->second.first;
		i++;
	}
	indices[generateIndex - continuous_point_adding_start_id].resize(k);
	indices_rew[generateIndex - continuous_point_adding_start_id].resize(k);
	//INFO("to "<<indices[generateIndex - continuous_point_adding_start_id].size());

	return max_used_nn;
}


template<>
int PRM<HeapPoint3D>::filter_nn(HeapNode<HeapPoint3D>* actual, std::vector<std::vector<int>> & indices, std::vector<std::vector<int>>& indices_rew,
		int generateIndex, int k, int k_search) {

}


