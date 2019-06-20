/*
 * HeuristicTypes.cpp
 *
 *  Created on: Feb 22, 2016
 *      Author: Robert Penicka
 */

#include "heuristic_types.h"
using crl::logger;

HeuristicTypes::HeuristicTypes() {

}

HeuristicTypes::~HeuristicTypes() {

}

bool operator==(const GraphNode<HeapPoint2D>& lhs, const GraphNode<HeapPoint2D>& rhs) {
	//WINFO("operator== " << lhs.x  <<" == "<< rhs.x);
	return lhs.data.x == rhs.data.x && lhs.data.y == rhs.data.y && lhs.reward == rhs.reward;
}

bool operator==(const GraphNode<HeapPoint2DHeading>& lhs, const GraphNode<HeapPoint2DHeading>& rhs) {
	//WINFO("operator== " << lhs.x  <<" == "<< rhs.x);
	return lhs.data.x == rhs.data.x && lhs.data.y == rhs.data.y && lhs.data.phi == rhs.data.phi && lhs.reward == rhs.reward;
}

std::ostream& operator <<(std::ostream &o, GraphNode<HeapPoint2D>& p) {
	std::cout.precision(6);
	o << std::fixed << " " << std::setprecision(6) << p.data.x << " " << std::setprecision(6) << p.data.y << " "
			<< std::setprecision(6) << p.reward;
	return o;
}

std::ostream& operator <<(std::ostream &o, GraphNode<HeapPoint2DHeading>& p) {
	std::cout.precision(6);
	o << std::fixed << " " << std::setprecision(6) << p.data.x << " " << std::setprecision(6) << p.data.y << " " << std::setprecision(6) << p.data.phi << " "
			<< std::setprecision(6) << p.reward;
	return o;
}

std::ostream& operator <<(std::ostream &o, const MapPoint& p) {
	std::cout.precision(6);
	o << std::fixed << " " << std::setprecision(6) << p.id << " " << std::setprecision(6) << p.x << " " << std::setprecision(6) << p.y << " "
			;
	return o;
}

/*
 bool operator>(GraphNode &dt1, GraphNode &dt2) {
 return dt1.reward > dt2.reward;
 }

 bool operator<(GraphNode &dt1, GraphNode &dt2) {
 return dt1.reward < dt2.reward;
 }
 */

TSP_EDGE_WEIGHT_TYPE parse_tsp_edge_type(std::string edge_type_string) {
	if ( !std::strcmp(edge_type_string.c_str(), TSP_EDGE_EUC_2D_STR) ) {
		return EUC_2D;
	} else if ( !std::strcmp(edge_type_string.c_str(), TSP_EDGE_CEIL_2D_STR) ) {
		return CEIL_2D;
	} else if ( !std::strcmp(edge_type_string.c_str(), TSP_EDGE_CEIL_EXPLICIT_STR) ) {
		return EXPLICIT;
	} else {
		ERROR("unknown tsp edge weight type "<<edge_type_string);
		exit(1);
	}
}

