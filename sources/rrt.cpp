/*
 * RRT.cpp
 *
 *  Created on: 16. 11. 2014
 *      Author: Robert Pěnička
 */

#include "rrt.h"
using crl::logger;

//Plan<PLANNER_STATE> RRT::plan(PLANNER_STATE * startPos, PLANNER_STATE * goalPos, std::vector<MeshObject*> obstacles,MeshObject* robot) {
//}

/*
 void RRT::fillPlan(Plan<PLANNER_STATE> * foundPlan) {
 TreeNode < PLANNER_STATE > *node = this->goal;
 while (node != NULL) {
 //std::cout << node->id << std::endl;
 foundPlan->addToPlan(node->getData());
 node = node->getParent();
 }
 }
 */

template<>
bool RRT<HeapPoint2D>::fillRandomState(TreeNode<HeapPoint2D> * positionToFill) {
	if (randDoubleMinMax(0, 1) <= GOAL_STATE_RANDOM_PROBABILITY_RRT) {
		positionToFill->city_node = false;
		positionToFill->data.x = goal->data.x;
		positionToFill->data.y = goal->data.y;
		INFO("random point "<<positionToFill->data.x<< " "<<positionToFill->data.y);
		return true;
	} else {
		positionToFill->city_node = false;
		positionToFill->data.x = randDoubleMinMax(rand_min_x, rand_max_x);
		positionToFill->data.y = randDoubleMinMax(rand_min_y, rand_max_y);
		INFO("random point "<<positionToFill->data.x<< " "<<positionToFill->data.y);
		return false;
	}
	//INFO("fill random state done")
}

template<>
bool RRT<HeapPoint2DHeading>::fillRandomState(TreeNode<HeapPoint2DHeading> * positionToFill) {
	if (randDoubleMinMax(0, 1) <= GOAL_STATE_RANDOM_PROBABILITY_RRT) {
		positionToFill->city_node = false;
		positionToFill->data.x = goal->data.x;
		positionToFill->data.y = goal->data.y;
		positionToFill->data.phi = goal->data.phi;
		positionToFill->data.radius = this->dubins_radius;
		return true;
	} else {
		positionToFill->city_node = false;
		positionToFill->data.x = randDoubleMinMax(rand_min_x, rand_max_x);
		positionToFill->data.y = randDoubleMinMax(rand_min_y, rand_max_y);
		positionToFill->data.phi = randDoubleMinMax(0, M_2PI);
		positionToFill->data.radius = this->dubins_radius;
		return false;
	}
	//INFO("fill random state HeapPoint2DHeading done")
}

template<>
bool RRT<HeapPoint2D>::hasCollision(std::vector<MeshObject*> obstacles, MeshObject* object,
		TreeNode<HeapPoint2D>* node) {
	bool freeBetween = true;
	Position3D object_position;
	object_position.x = node->data.x;
	object_position.y = node->data.y;
	object_position.z = 0;
	object_position.yaw = 0;
	object_position.pitch = 0;
	object_position.roll = 0;
	return MeshObject::collide(&obstacles, robot, object_position);
}

template<>
bool RRT<HeapPoint2DHeading>::hasCollision(std::vector<MeshObject*> obstacles, MeshObject* object,
		TreeNode<HeapPoint2DHeading>* node) {
	bool freeBetween = true;
	Position3D object_position;
	object_position.x = node->data.x;
	object_position.y = node->data.y;
	object_position.z = 0;
	object_position.yaw = node->data.phi;
	object_position.pitch = 0;
	object_position.roll = 0;
	return MeshObject::collide(&obstacles, robot, object_position);
}

template<>
double RRT<HeapPoint2D>::getPointFromNN(TreeNode<HeapPoint2D> * newNodeRandom, TreeNode<HeapPoint2D> * nearestNeighbor,
		double distance) {
	if (distance > collision_distance_check) {
		//need to find point on line bwtween NN and random node
		double distanceMaxChanger = distance / collision_distance_check;
		//find position on line between newNode and nearestNeighbor that is in distance MAX_DISTANCE_INCREASE_M
		newNodeRandom->data.x = nearestNeighbor->data.x
				+ (newNodeRandom->data.x - nearestNeighbor->data.x) / distanceMaxChanger;
		newNodeRandom->data.y = nearestNeighbor->data.y
				+ (newNodeRandom->data.y - nearestNeighbor->data.y) / distanceMaxChanger;
		return collision_distance_check;
	}
	return distance;
}

template<>
double RRT<HeapPoint2DHeading>::getPointFromNN(TreeNode<HeapPoint2DHeading> * newNodeRandom,
		TreeNode<HeapPoint2DHeading> * nearestNeighbor, double distance) {
	if (distance > collision_distance_check) {
		//need to find point on line bwtween NN and random node
		double distanceMaxChanger = distance / collision_distance_check;
		double t, t1, distanceAfter;
		//find position on line between newNode and nearestNeighbor that is in distance MAX_DISTANCE_INCREASE_M
		newNodeRandom->data.x = nearestNeighbor->data.x
				+ (newNodeRandom->data.x - nearestNeighbor->data.x) / distanceMaxChanger;
		newNodeRandom->data.y = nearestNeighbor->data.y
				+ (newNodeRandom->data.y - nearestNeighbor->data.y) / distanceMaxChanger;

		t = fabs(newNodeRandom->data.phi - nearestNeighbor->data.phi);
		if (t < M_2PI - t) {
			//normal rotation between two angles
			//newNodeRandom->getData().setYaw(
			//		normalizeAngle(
			//				nearestNeighbor->getData().getYaw()
			//						+ (newNodeRandom->getData().getYaw() - nearestNeighbor->getData().getYaw())
			//								/ distanceMaxChanger));
			newNodeRandom->data.phi = normalizeAngle(
					nearestNeighbor->data.phi
							+ (newNodeRandom->data.phi - nearestNeighbor->data.phi) / distanceMaxChanger);
		} else {
			//reversed rotation between two angles
			//newNodeRandom->getData().setYaw(
			//		normalizeAngle(
			//				nearestNeighbor->getData().getYaw()
			//						+ (M_2PI - newNodeRandom->getData().getYaw() - nearestNeighbor->getData().getYaw())
			//								/ distanceMaxChanger));
			newNodeRandom->data.phi = normalizeAngle(
					nearestNeighbor->data.phi
							+ (M_2PI - newNodeRandom->data.phi - nearestNeighbor->data.phi) / distanceMaxChanger);

		}
		return collision_distance_check;
	}
	return distance;
}

/*
 void RRT::getPointFromNN(TreeNode<AUV_State> * goalPos, TreeNode<AUV_State> * nearestNeighbor, double distance) {
 Dubins3D dubinsPath(MAX_TURN_RADIUS, MAX_TURN_RADIUS, 1);
 PlanDubins3D plan = dubinsPath.getShortestPlan(
 Position3D(nearestNeighbor->getData().output.Out1x, nearestNeighbor->getData().output.Out2y,
 nearestNeighbor->getData().output.Out3z, nearestNeighbor->getData().output.Out6rz, 0, 0),
 Position3D(goalPos->getData().output.Out1x, goalPos->getData().output.Out2y,
 goalPos->getData().output.Out3z, goalPos->getData().output.Out6rz, 0, 0), MAX_DISTANCE_INCREASE);
 if (plan.plan.size() > 1) {
 AUV_State stateAfterFirst;
 stateAfterFirst.output.Out1x = plan.plan.get(1).getX();
 stateAfterFirst.output.Out2y = plan.plan.get(1).getY();
 stateAfterFirst.output.Out3z = plan.plan.get(1).getZ();
 stateAfterFirst.output.Out4rx = 0;
 stateAfterFirst.output.Out5ry = 0;
 stateAfterFirst.output.Out6rz = plan.plan.get(1).getYaw();
 stateAfterFirst.planned_type = plan.type;
 goalPos->setData(stateAfterFirst);
 }
 }
 */

/*
 double RRT::calcDistance(TreeNode<Position3D> * newNodeRandom, TreeNode<Position3D> * nearestNeighbor) {
 double distanceAfter, t, t1;
 distanceAfter = 0;
 distanceAfter += POW(newNodeRandom->getData().getX() - nearestNeighbor->getData().getX());
 distanceAfter += POW(newNodeRandom->getData().getY() - nearestNeighbor->getData().getY());
 distanceAfter += POW(newNodeRandom->getData().getZ() - nearestNeighbor->getData().getZ());
 t = fabs(newNodeRandom->getData().getYaw() - nearestNeighbor->getData().getYaw());
 t1 = MIN(t, M_2PI - t);
 //std::cout << "t , t1 Yaw" << t << " , " << t1 << std::endl;
 distanceAfter += POW(t1);
 t = fabs(newNodeRandom->getData().getPitch() - nearestNeighbor->getData().getPitch());
 t1 = MIN(t, M_2PI - t);
 //std::cout << "t , t1 Pitch" << t << " , " << t1 << std::endl;
 distanceAfter += POW(t1);
 t = fabs(newNodeRandom->getData().getRoll() - nearestNeighbor->getData().getRoll());
 t1 = MIN(t, M_2PI - t);
 //std::cout << "t , t1 Roll" << t << " , " << t1 << std::endl;
 distanceAfter += POW(t1);
 distanceAfter = sqrt(distanceAfter);
 return distanceAfter;
 }
 */

