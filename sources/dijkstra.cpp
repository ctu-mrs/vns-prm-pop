/*
 * Dijkstra.cpp
 *
 *  Created on: 2. 2. 2015
 *      Author: Robert Pěnička
 */

#include "dijkstra.h"

template<>
std::ostream& operator <<(std::ostream &o, const HeapNode<HeapPoint2D>& p) {
	o << p.node_id << " cid " << p.cluster_id << " x=" << p.data.x << " y=" << p.data.y << " city_node=" << p.city_node;
	return o;
}

template<>
std::ostream& operator <<(std::ostream &o, const HeapNode<HeapPoint2DHeading>& p) {
	o << p.node_id << " cid " << p.cluster_id << " x=" << p.data.x << " y=" << p.data.y << " phi=" << p.data.phi << " city_node=" << p.city_node;
	return o;
}

template<>
std::ostream& operator <<(std::ostream &o, const HeapNode<HeapPoint3D>& p) {
	o << p.node_id << " cid " << p.cluster_id << " x=" << p.data.x << " y=" << p.data.y << " z=" << p.data.z<< " city_node=" << p.city_node;
	return o;
}

