/*
 * dop.h
 *
 *  Created on: Apr 30, 2016
 *      Author: Robert Penicka, Petr Vana
 */

#ifndef VNS_OBST_OP_PATH_H_
#define VNS_OBST_OP_PATH_H_

#include <crl/alg/algorithm.h>
#include "opendubins/dubins.h"
#include "prm.h"
#include "heuristic_types.h"
#include "crl/logging.h"
#include "math_common.h"
#include <tuple>

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

#define VARIABLE_STR(s) #s
#define STR(s) VARIABLE_STR(s)
#define ROVNASE1(X) X =
#define ROVNASE(X)  ROVNASE1(X)

#define INFO_VAR(x) INFO( STR(x) << " = " <<  x )
#define INFO_VAR_RED(x) INFO_RED( STR(x) << " = " <<  x )

// Class for the Dubins Orienteering Problem
// There is are two fixed positions - start and end
// Departure and termination headings are not given
// unlimited number of points can be inserted between start and end

template<class T>
class VnsSopPath {
public:
	VnsSopPath() {

	}

	VnsSopPath(int startClusterIdx_, int endClusterIdx_, GOPTYPE gop_type);

	static void setSamplesDistances(std::vector<std::vector<GraphNode<T>>>* nodesAllClusters_,
			samples_distances_type* distances_, cluster_bound_distances* clustersMinDistances_,
			cluster_bound_distances* clustersMaxDistances_);
	static std::vector<std::vector<GraphNode<T>>> *getAllSamples();
	static void setPRM(PRM<T> * prm);

	std::vector<int> getAllTargets();

	virtual ~VnsSopPath();

	// returns number of Dubins maneuvers (targets.size() + 1)
	int getSize() const;

	// returns all Dubins maneuver in the shortest found path
	std::vector<IndexSOP> getPath() const;

	std::vector<GraphNode<T>> getPathNeighAngIds(GraphNode<T> & chainGeneratingNeighAngEnd,
			bool fillchainGenNA = false) const;

	// returns only the distance of the shortest found path
	// it schould be slightly faster than using getPath() and sum the distance
	double getPathLength(const GraphNode<T> & chainGeneratingNeighAngFrom = GraphNode<T>()) const;
	double getPathLengthRear(const GraphNode<T> & chainGeneratingNeighAngFrom = GraphNode<T>()) const;
	double getPathLengthCalculate(const GraphNode<T> & chainGeneratingNeighAngFrom = GraphNode<T>()) const;
	double getPathLengthCalculateRear(const GraphNode<T> & chainGeneratingNeighAngStart = GraphNode<T>()) const;

	// add new target to specific position
	// example: addPoint(newPoint, 0)
	// before:  start > t1 > end
	// after:   start > newPoint > t1 > end
	void addPoint(int clusterId, int idx);

	// add new targets to specific position
	// example: addPoint(points, 0)
	// before:  start > t1 > end
	// after:   start > points[0] >  points[last] > t1 > end
	void addPoint(std::vector<int> clusterIds, int idx);

	// add point to the best position
	void addPoint(int clusterId);

	// remove one target with specific index
	// idx is in intervals [0, getSize()-2] = [0, getNumTargets()-1] = [0, targets.size()]
	void removePoint(int idx);

	// remove targets starting with idxStart and ending with idxStop
	void removePoint(int idxStart, int idxStop);

	// same as addPoint but it does not insert the point
	// idx is in intervals [0, getSize()] = [0, targets.size()+1]
	// return length of new path
	double tryToAdd(int cluster_id, int idx) const;

	double tryToAdd(std::vector<int> p, int idx) const;

	//replaces single target at idx with GraphNode p
	double tryToReplace(int clusterId, int idx, int numToReplace = 1) const;

	//replaces single target at idx with vector of targets p
	double tryToReplace(std::vector<int> clusterIds, int idx) const;

	//returns length of path with exchanged idx1 and idx2
	double tryToExchange(int idx1, int idx2);

	//returns length of path with moved idxWhatToMove to idxWhereToMove
	double tryToMove(int idxWhatToMove, int idxWhereToMove);

	// same as removePoint but it does not remove the point
	// idx is in intervals [0, getSize()-2] = [0, getNumTargets()-1] = [0, targets.size()]
	// return length of new path
	double tryToRemove(int idx) const;

	double tryToRemove(int idxStart, int idxStop) const;

	// remove the worst point
	void removeOne();

	// test 2 opt operation between idx1 and idx2
	// idxFrom and idxTo are in intervals [0, getNumTargets()-1] = [0, targets.size()-1]
	// the path idxFrom - idxTo is inserted in reverse mode
	// return length of new path
	// reverse path between idx1+1 and idx2
	double tryTwoOpt(int idxFrom, int idxTo);

	//performs the two opt
	// idxFrom and idxTo are in intervals [0, getNumTargets()-1] = [0, targets.size()-1]
	// the path idxFrom - idxTo is inserted in reverse mode
	void twoOpt(int idxFrom, int idxTo);

	//return number of targets - number of nodes between start a and end
	int getNumTargets();

	//returns target at position idx - idx in intervals [0, targets.size-1]
	int getTarget(int idx);

	//return target subvector
	std::vector<int> getTargets(int idxFrom, int idxTo);

	//return samples
	std::vector<std::vector<GraphNode<T>>> getSamples();
	std::vector<std::vector<GraphNode<T>>>& getSamplesRef();
	shortest_matrix & getShortestRef();
	shortest_matrix & getShortestBackRef();
	//return number of samples - number of nodes in path totally = with start and end
	int getNumSamples();

	void checkShortestConsistency();
	void checkSameSamples(std::string text = std::string(""));

	//prints out ids of nodes
	void listIds();
	void listNodeIds();
	void listRewads();
	void printShortestDistances();

	std::vector<int>::iterator targetsBegin();
	std::vector<int>::iterator targetsEnd();

	void update();

	std::vector<T> getPathSampled(double samplePathDistance);

	double getReward();
	static samples_distances_type* allDistances;
	static std::vector<std::vector<GraphNode<T>>>* allSamples;
	static cluster_bound_distances* clustersMinDistances;
	static cluster_bound_distances* clustersMaxDistances;
	static PRM<T> * prm;

protected:

	//static void generateSamples(single_cluster_samples & samples, std::vector<GraphNode> p, int clusterID, int startClusterIdx_, int endClusterIdx_);

	void updateAfterInsert(int idxStart, int idxEnd);
	void updateAfterRemove(int idxStart, int idxEnd);

	int startClusterIdx;
	int endClusterIdx;

	std::vector<int> targets;

	// all samples
	// [1st index] - choose point/target
	// [2nd index] - choose neighborhood point in target
	// [3nd index] - choose state (sample) for this neighborhood
	std::vector<std::vector<GraphNode<T>>> samples;

	// find all shortest paths
	// matrix:
	//    1) to cluster (layer)
	//    2) to cluster node (layer)
	shortest_matrix shortest;

	// find all shortest paths - backward path
	// matrix:
	//    1) to cluster (layer)
	//    2) to cluster node (layer)
	shortest_matrix shortest_back;

	GOPTYPE gop_type;

};

#define MEMORIZE_ALL_DISTANCES
#define M (numeric_limits<double>::max())

using namespace std;
using namespace crl;
using namespace crl::gui;
using namespace opendubins;
using crl::logger;

template<class T>
std::vector<std::vector<GraphNode<T>>>* VnsSopPath<T>::allSamples = NULL;
template<class T>
samples_distances_type* VnsSopPath<T>::allDistances = NULL;
template<class T>
cluster_bound_distances* VnsSopPath<T>::clustersMinDistances = NULL;
template<class T>
cluster_bound_distances* VnsSopPath<T>::clustersMaxDistances = NULL;
template<class T>
PRM<T> * VnsSopPath<T>::prm = NULL;

//std::vector<std::vector<std::vector<std::vector<std::vector<std::vector<int>>>>> >VNSGOPPath::allDistancesUsage;

template<class T>
VnsSopPath<T>::VnsSopPath(int startClusterIdx_, int endClusterIdx_, GOPTYPE gop_type_) :
		startClusterIdx(startClusterIdx_), endClusterIdx(endClusterIdx_) {
	INFO("VNSGOPPath constructor begin");
	this->gop_type = gop_type_;
	update();
	INFO("VNSGOPPath constructor end");
}

template<class T>
VnsSopPath<T>::~VnsSopPath() {

}

template<class T>
void VnsSopPath<T>::setSamplesDistances(std::vector<std::vector<GraphNode<T>>> * nodesAllClusters_,
		samples_distances_type * distances_, cluster_bound_distances * clustersMinDistances_,
		cluster_bound_distances * clustersMaxDistances_) {
	INFO("set allSamples size "<<nodesAllClusters_->size())
	VnsSopPath::allSamples = nodesAllClusters_;
	INFO("set allDistances size "<<distances_->size())
	VnsSopPath::allDistances = distances_;
	INFO("set clustersMinDistances size "<<clustersMinDistances_->size())
	VnsSopPath::clustersMinDistances = clustersMinDistances_;
	INFO("set clustersMaxDistances size "<<clustersMaxDistances_->size())
	VnsSopPath::clustersMaxDistances = clustersMaxDistances_;
	INFO("end setSamplesDistances");
}

template<class T>
void VnsSopPath<T>::setPRM(PRM<T> * prm) {
	VnsSopPath<T>::prm = prm;
}

template<class T>
std::vector<std::vector<GraphNode<T>>>* VnsSopPath<T>::getAllSamples() {
	return allSamples;
}

template<class T>
std::vector<int> VnsSopPath<T>::getAllTargets() {
	return targets;
}

template<class T>
int VnsSopPath<T>::getSize() const {

	return targets.size() + 1;
}

template<class T>
void VnsSopPath<T>::addPoint(int clusterId, int idx) {
	if (idx > targets.size() || idx < 0) {
		INFO_RED("addPoint idx "<<idx<<" exceeds maximum "<<(targets.size()-1));
		exit(1);
	}

	targets.insert(targets.begin() + idx, clusterId);

	updateAfterInsert(idx, idx);

	double length = getPathLength();
	double length_rear = getPathLengthRear();

	if (fabs(length - length_rear) > 0.01) {
		ERROR("error in updateAfterInsert");
		INFO_VAR(length);
		INFO_VAR(length_rear);
		exit(1);
	}
}

template<class T>
void VnsSopPath<T>::addPoint(std::vector<int> clusterIds, int idx) {
	if (idx > targets.size() || idx < 0) {
		INFO_RED("addPoint idx "<<idx<<" exceeds maximum "<<(targets.size()-1));
		exit(1);
	}
	targets.insert(targets.begin() + idx, clusterIds.begin(), clusterIds.end());
	updateAfterInsert(idx, idx + clusterIds.size() - 1); //min len 68.7228      17
}

template<class T>
void VnsSopPath<T>::addPoint(int clusterId) {

	int idx = 0;
	double minLen = numeric_limits<double>::max();
	const int S = getSize();

	for (int i = 0; i < S; i++) {
		double newLen = tryToAdd(clusterId, i);
		if (newLen < minLen) {
			minLen = newLen;
			idx = i;
		}
	}
	addPoint(clusterId, idx);
}

template<class T>
void VnsSopPath<T>::removePoint(int idx) {
	if (idx >= targets.size() || idx < 0) {
		INFO_RED("removePoint idx "<<idx<<" exceeds maximum "<<(targets.size()-1));
		exit(1);
	}
	double lengthBef = getPathLength();
	targets.erase(targets.begin() + idx);

	updateAfterRemove(idx, idx);
}

template<class T>
void VnsSopPath<T>::removePoint(int idxStart, int idxStop) {
	targets.erase(targets.begin() + idxStart, targets.begin() + idxStop + 1);
	updateAfterRemove(idxStart, idxStop); // - min len 68.7228      18
}

template<class T>
double VnsSopPath<T>::tryToAdd(int cluster_id, int idx) const {
	if (idx > targets.size() || idx < 0) {
		INFO_RED("tryToAdd idx "<<idx<<" exceeds maximum "<<(targets.size()-1));
		exit(1);
	}
	//INFO_VAR(idx);
	std::vector<GraphNode<T>> fromSamples = samples[idx];
	std::vector<GraphNode<T>> toSamples = samples[idx + 1];

	single_cluster_shortest_matrix fromLen = shortest[idx];
	single_cluster_shortest_matrix toLen = shortest_back[idx + 1];

	std::vector<GraphNode<T>> actSamples = (*allSamples)[cluster_id];

	const int fromSamplesSize = fromSamples.size();

	// find shortest paths to generate samples
	std::vector<double> actLen;
	actLen.assign(actSamples.size(), M);
	//INFO("find to");
	for (int fromNeighID = 0; fromNeighID < fromSamples.size(); ++fromNeighID) {
		GraphNode<T> & from = fromSamples[fromNeighID];
		double fromlength = fromLen[fromNeighID].distance;
		//INFO_VAR(fromlength);
		for (int toNeighID = 0; toNeighID < actSamples.size(); ++toNeighID) {
			GraphNode<T> & to = actSamples[toNeighID];
			//double dl = Dubins(from.toState(), to.toState(), radius).length;

			double dl = (*allDistances)[from.cluster_id][to.cluster_id][fromNeighID][toNeighID];
			//INFO_VAR(dl);
			double nl = dl + fromlength;

			if (nl < actLen[toNeighID]) {
				actLen[toNeighID] = nl;
			}
		}
	}
	//INFO("find from");
	// find the shortest path from actual samples to next <toSamples>
	double minLen = M;
	for (int toNeighID = 0; toNeighID < toSamples.size(); ++toNeighID) {
		GraphNode<T> & to = toSamples[toNeighID];
		double tolength = toLen[toNeighID].distance;
		//INFO_VAR(tolength);
		for (int fromNeighID = 0; fromNeighID < actSamples.size(); ++fromNeighID) {
			GraphNode<T> & from = actSamples[fromNeighID];

			double dl = (*allDistances)[from.cluster_id][to.cluster_id][fromNeighID][toNeighID];
			//INFO_VAR(dl);
			double nl = dl + tolength + actLen[fromNeighID];
			if (nl < minLen) {
				minLen = nl;
			}
		}
	}
	//INFO_VAR(minLen);
	//INFO("tryToAdd single end");
	return minLen;
}

template<class T>
double VnsSopPath<T>::tryToAdd(std::vector<int> clusterIds, int idx) const {
//INFO("tryToAdd vector begin");
	if (idx > targets.size() || idx < 0) {
		INFO_RED("tryToAdd idx "<<idx<<" exceeds maximum "<<(targets.size()-1));
		exit(1);
	}
	std::vector<GraphNode<T>> fromSamples = samples[idx];
	std::vector<GraphNode<T>> toSamples = samples[idx + 1];

	single_cluster_shortest_matrix fromLen = shortest[idx];
	single_cluster_shortest_matrix toLen = shortest_back[idx + 1];

	const int addingNumPoints = clusterIds.size();
	//INFO("tryToAdd vector size "<<addingNumPoints);

	// generate samples
	std::vector<std::vector<GraphNode<T>>> actSamples(addingNumPoints);
	for (int var = 0; var < clusterIds.size(); ++var) {
		//generateSamples(actSamples[var], p[var], resolution);
		actSamples[var] = (*allSamples)[clusterIds[var]];
	}

	shortest_matrix shortestadded;
	shortestadded.resize(addingNumPoints + 2);
	shortestadded[0].resize(fromLen.size());
	int fromClusterId = fromSamples[0].cluster_id;
	for (int target = 1; target < addingNumPoints + 2; target++) {
		shortestadded[target].assign((*allDistances)[fromClusterId][clusterIds[target - 1]].size(),
				ClusterNodeDist(-1, M));

		for (int neighID1 = 0; neighID1 < (*allDistances)[fromClusterId][clusterIds[target - 1]].size(); ++neighID1) {
			for (int neighID2 = 0; neighID2 < (*allDistances)[fromClusterId][clusterIds[target - 1]][neighID1].size();
					++neighID2) {

				auto len1 = shortestadded[target - 1][neighID1].distance;
				auto len2 = (*allDistances)[fromClusterId][clusterIds[target - 1]][neighID1][neighID2];
				auto nLen = len1 + len2;
				auto &len = shortestadded[target][neighID2];
				if (nLen < len.distance) {
					len.idClusterNode = neighID1;
					len.distance = nLen;
				}
			}
			fromClusterId = clusterIds[target - 1];
		}
	}

	// find the shortest path from shortestadded and
	double minLen = M;
	for (int neighID2 = 0; neighID2 < shortestadded[addingNumPoints + 1].size(); neighID2++) {
		double nl = shortestadded[addingNumPoints + 1][neighID2].distance + toLen[neighID2].distance;
		if (nl < minLen) {
			minLen = nl;
		}
	}
	//INFO("tryToAdd vector begin");
	return minLen;
}

template<class T>
double VnsSopPath<T>::tryToReplace(std::vector<int> clusterIds, int idx) const {
	//INFO("tryToReplace multi begin");
	std::vector<GraphNode<T>> fromSamples = samples[idx];
	std::vector<GraphNode<T>> toSamples = samples[idx + 2];

	single_cluster_shortest_matrix fromLen = shortest[idx];
	single_cluster_shortest_matrix toLen = shortest_back[idx + 2];

	const int addingNumPoints = clusterIds.size();
	//INFO("tryToAdd vector size "<<addingNumPoints);

	// generate samples
	std::vector<std::vector<GraphNode<T>>> actSamples(addingNumPoints);
	for (int var = 0; var < clusterIds.size(); ++var) {
		//generateSamples(actSamples[var], p[var], resolution);
		actSamples[var] = (*allSamples)[clusterIds[var]];
	}

	shortest_matrix shortestadded;
	shortestadded.resize(addingNumPoints + 2);
	shortestadded[0].resize(fromLen.size());

	int fromClusterId = fromSamples[0].cluster_id;
	for (int target = 1; target < addingNumPoints + 2; target++) {
		//INFO("target "<<target);
		shortestadded[target].assign((*allDistances)[fromClusterId][clusterIds[target]][0].size(),
				ClusterNodeDist(-1, M));

		for (int neighID1 = 0; neighID1 < (*allDistances)[fromClusterId][clusterIds[target - 1]].size(); ++neighID1) {
			for (int neighID2 = 0; neighID2 < (*allDistances)[fromClusterId][clusterIds[target - 1]][neighID1].size();
					++neighID2) {

				auto len1 = shortestadded[target - 1][neighID1].distance;
				auto len2 = (*allDistances)[fromClusterId][clusterIds[target - 1]][neighID1][neighID2];
				//INFO("Shortest len1 "<<len1<<" len2 "<<len2);
				auto nLen = len1 + len2;
				auto &len = shortestadded[target][neighID2];
				if (nLen < len.distance) {
					len.idClusterNode = neighID1;
					len.distance = nLen;
					//INFO("shortest path to "<<idx2<<"  "<<len.second);
				}
			}
		}
		fromClusterId = clusterIds[target - 1];
	}

	// find the shortest path from shortestadded and
	double minLen = M;
	for (int neighID2 = 0; neighID2 < shortestadded[addingNumPoints + 1].size(); neighID2++) {
		double nl = shortestadded[addingNumPoints + 1][neighID2].distance + toLen[neighID2].distance;
		if (nl < minLen) {
			minLen = nl;
		}
	}
	//INFO("tryToReplace multi end");
	return minLen;
}

template<class T>
double VnsSopPath<T>::tryToReplace(int clusterId, int idx, int numToReplace) const {
//INFO("tryToReplace single begin");
	const int S = getSize();
	bool replaceLast = false;
	if (idx + numToReplace == S) {
		replaceLast = true;
		//INFO("try to replace last");
	} else if (idx >= S) {
		INFO_RED("tryToReplace idx "<<idx<<" exceeds maximum "<<S);
		exit(1);
	}

	std::vector<GraphNode<T>> fromSamples = samples[idx];
	single_cluster_shortest_matrix fromLen = shortest[idx];
	std::vector<GraphNode<T>> replaceWSamples = (*allSamples)[clusterId];

	const int fromSamplesSize = fromSamples.size();

	// find shortest paths to generate samples
	std::vector<double> newToDistances;
	newToDistances.assign(replaceWSamples.size(), M);

	//INFO("calc to");
	for (int fromNeighID = 0; fromNeighID < fromSamples.size(); ++fromNeighID) {
		GraphNode<T> & from = fromSamples[fromNeighID];
		for (int toNeighID = 0; toNeighID < replaceWSamples.size(); ++toNeighID) {
			GraphNode<T> & to = replaceWSamples[toNeighID];

			//double dl = Dubins(from.toState(), to.toState(), radius).length;
			double dl = (*allDistances)[from.cluster_id][to.cluster_id][fromNeighID][toNeighID];
			double nl = dl + fromLen[fromNeighID].distance;
			if (nl < newToDistances[toNeighID]) {
				newToDistances[toNeighID] = nl;
			}
		}
	}
	double minLen = M;
	//INFO("calc from");
	std::vector<GraphNode<T>> toSamples = samples[idx + 1 + numToReplace];
	single_cluster_shortest_matrix toLen = shortest_back[idx + 1 + numToReplace];
	// find the shortest path from actual samples to next <toSamples>

	for (int fromNeighID = 0; fromNeighID < replaceWSamples.size(); ++fromNeighID) {
		GraphNode<T> & from = replaceWSamples[fromNeighID];
		for (int toNeighID = 0; toNeighID < toSamples.size(); ++toNeighID) {
			GraphNode<T> & to = toSamples[toNeighID];
			//double dl = Dubins(from.toState(), to.toState(), radius).length;
			double dl = (*allDistances)[from.cluster_id][to.cluster_id][fromNeighID][toNeighID];
			double nl = dl + toLen[toNeighID].distance + newToDistances[fromNeighID];
			if (nl < minLen) {
				minLen = nl;
			}
		}
	}

	//INFO("tryToReplace single end");
	return minLen;
}

template<class T>
double VnsSopPath<T>::tryToRemove(int idx) const {
//INFO("tryToRemove single begin");
	double minLen = M;
	for (int fromNeighID = 0; fromNeighID < samples[idx].size(); ++fromNeighID) {
		auto from = samples[idx][fromNeighID];
		auto fromLen = shortest[idx][fromNeighID];
		for (int toNeighID = 0; toNeighID < samples[idx + 2].size(); ++toNeighID) {
			auto to = samples[idx + 2][toNeighID];
			auto toLen = shortest_back[idx + 2][toNeighID];

			double dl = (*allDistances)[from.cluster_id][to.cluster_id][fromNeighID][toNeighID];
			double nl = dl + toLen.distance + fromLen.distance;
			if (nl < minLen) {
				minLen = nl;
			}

		}
	}
	//INFO("tryToRemove single end");
	return minLen;
}

template<class T>
double VnsSopPath<T>::tryToRemove(int idxStart, int idxStop) const {
//INFO("tryToRemove multi begin");
	double minLen = M;
	for (int fromNeighID = 0; fromNeighID < shortest[idxStart].size(); ++fromNeighID) {
		auto fromLen = shortest[idxStart][fromNeighID];
		auto from = samples[idxStart][fromNeighID];
		for (int toNeighID = 0; toNeighID < shortest_back[idxStop + 2].size(); ++toNeighID) {
			auto toLen = shortest_back[idxStop + 2][toNeighID];
			auto to = samples[idxStop + 2][toNeighID];
			double dl = (*allDistances)[from.cluster_id][to.cluster_id][fromNeighID][toNeighID];
			double nl = dl + toLen.distance + fromLen.distance;
			if (nl < minLen) {
				minLen = nl;
			}
		}
	}
//INFO("tryToRemove multi end");
	return minLen;
}

template<class T>
void VnsSopPath<T>::removeOne() {
	int idx = 0;
	double minLen = numeric_limits<double>::max();
	const int S = getSize();

	for (int i = 0; i < S - 1; i++) {
		double newLen = tryToRemove(i);
		if (newLen < minLen) {
			minLen = newLen;
			idx = i;
		}
	}

	removePoint(idx);
}

template<class T>
double VnsSopPath<T>::tryToExchange(int idx1, int idx2) {
	//INFO("tryToExchange multi begin");
	if (idx1 > targets.size() || idx1 < 0 || idx2 > targets.size() || idx2 < 0 || abs(idx1 - idx2) < 2) {
		INFO_RED("tryToExchange idx "<<idx1<<" or "<<idx2<<" exceeds maximum "<<(targets.size()-1));
		exit(1);
	}
	//move target idxFrom before idxTo and
	int idxFrom = MIN(idx1, idx2);
	int idxTo = MAX(idx1, idx2);

	//INFO_GREEN("tryToExchange from " << idxFrom << " to " << idxTo);

	//exit when dist idxStart to idxEnd < 2 ---- need minimally distance of two*
	//reverse path between idx1+1 and idx2, so need
	//ensure idxStart <= idxEnd
	if (idxTo - idxFrom < 2) {
		INFO_RED("idxEnd and idxStart are too close "<<(idxTo - idxFrom)<<" must be lower than 2");
		exit(1);
	}
	if (idxTo >= targets.size()) {
		INFO_RED("idxEnd+1 is out of samples array "<<idxTo<<" samples number "<<targets.size());
		exit(1);
	}

	//change indexing from targets to samples
	int idxStartSamples = idxFrom + 1;
	int idxEndSamples = idxTo + 1;

	int i = idxStartSamples - 1;
	int j = idxEndSamples;
	std::vector<GraphNode<T>> samples_I = samples[i];	//i
	std::vector<GraphNode<T>> samples_I_p = samples[i + 1];	//i+1
	std::vector<GraphNode<T>> samples_J_m = samples[j - 1];	//j -1
	std::vector<GraphNode<T>> samples_J = samples[j];	//j
	std::vector<GraphNode<T>> samples_J_p = samples[j + 1];	//j+1
	std::vector<GraphNode<T>> samples_I_pp = samples[i + 2];	//i+2
	int idI = samples[i][0].cluster_id;
	int idI_p = samples[i + 1][0].cluster_id;
	int idI_pp = samples[i + 2][0].cluster_id;
	int idJ_m = samples[j - 1][0].cluster_id;
	int idJ = samples[j][0].cluster_id;
	int idJ_p = samples[j + 1][0].cluster_id;

	//new path is i -> j -> i+2 -> ... -> j-1 -> i+1 -> j+1

	single_cluster_shortest_matrix normalOrderTotLen = shortest[i];	// need to use idxStart+1 as to the

	single_cluster_shortest_matrix normalOrderFromLen = shortest_back[j + 1];

	shortest_matrix pathPartShortest;
	int numCityBetween = (idxEndSamples - idxStartSamples + 2);
	pathPartShortest.resize(numCityBetween);
	pathPartShortest[0] = normalOrderTotLen;	// set actual shortest distance to idxStart

	int idxPathPartShortest = 1;

	pathPartShortest[idxPathPartShortest].assign(samples_J.size(), ClusterNodeDist(-1, M));

	for (int neighID1 = 0; neighID1 < samples_I.size(); ++neighID1) {
		for (int neighID2 = 0; neighID2 < samples_J.size(); ++neighID2) {
			auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
			auto len2 = (*allDistances)[idI][idJ][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]
			auto nLen = len1 + len2;
			auto &len = pathPartShortest[idxPathPartShortest][neighID2]; // actual shortest path from start to [target][idx2]
			if (nLen < len.distance) {
				len.idClusterNode = neighID1;
				len.distance = nLen;
			}
		}
	}

	idxPathPartShortest++; //2
	//INFO("calc shortest from idJ "<< idJ <<" to idI_pp"<< idI_pp);
	//pathPartShortest[idxPathPartShortest].assign(reverseOrderFromSamples.size(), dst_pair(-1, M));
	pathPartShortest[idxPathPartShortest].assign(samples_I_pp.size(), ClusterNodeDist(-1, M));
	for (int neighID1 = 0; neighID1 < samples_J.size(); ++neighID1) {
		for (int neighID2 = 0; neighID2 < samples_I_pp.size(); ++neighID2) {
			auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
			auto len2 = (*allDistances)[idJ][idI_pp][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]
			auto nLen = len1 + len2;
			auto &len = pathPartShortest[idxPathPartShortest][neighID2]; // actual shortest path from start to [target][idx2]
			if (nLen < len.distance) {
				len.idClusterNode = neighID1;
				len.distance = nLen;
			}
		}
	}

	idxPathPartShortest++; //3
	//calc from j to i+1
	for (int target = idxStartSamples + 1; target <= idxEndSamples - 2; target++) {
		//INFO("----------------");
		//pathPartShortest[idxPathPartShortest].assign(samples[target].size(), dst_pair(-1, M));
		int idFrom = target;
		int idTo = target + 1;
		pathPartShortest[idxPathPartShortest].assign(samples[idTo].size(), ClusterNodeDist(-1, M));

		//INFO("----calc shortest from "<< samples[idFrom][0].node.id <<" to "<< samples[idTo][0].node.id);
		//INFO("idFrom " << samples[idFrom][0].node.id);
		//INFO("idto "<< samples[idTo][0].node.id);

		const int S1_neight = samples[idFrom].size();	// from neigh
		const int S2_neight = samples[idTo].size();	//to neigh
		for (int neighID1 = 0; neighID1 < S1_neight; ++neighID1) {
			for (int neighID2 = 0; neighID2 < S2_neight; ++neighID2) {

				auto & from = samples[idFrom][neighID1];
				auto & to = samples[idTo][neighID2];

				auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
				//auto len2 = distances[idTo][getInverseId(idx2)][getInverseId(idx1)]; //need to use idto because of indexing of distances
				auto len2 = (*allDistances)[from.cluster_id][to.cluster_id][neighID1][neighID2];

				auto nLen = len1 + len2;
				//INFO(idx1<<" "<<idx2 <<" len1 "<<len1<<" len2 "<<len2<<" nLen "<<nLen);
				//INFO(idx1<<" "<<idx2<<" len "<<len1<<" dist "<< distances[idTo][idx2][idx1]);
				auto &len = pathPartShortest[idxPathPartShortest][neighID2];// actual shortest path from start to [target][idx2]

				if (nLen < len.distance) {
					len.idClusterNode = neighID1;
					len.distance = nLen;
				}
			}
		}

		idxPathPartShortest++;
	}

	//INFO("calc shortest from idJ_m "<< idJ_m <<" to idI_p "<< idI_p);
	//pathPartShortest[idxPathPartShortest].assign(reverseOrderFromSamples.size(), dst_pair(-1, M));
	pathPartShortest[idxPathPartShortest].assign(samples_I_p.size(), ClusterNodeDist(-1, M));

	for (int neighID1 = 0; neighID1 < samples_J_m.size(); ++neighID1) {
		for (int neighID2 = 0; neighID2 < samples_I_p.size(); ++neighID2) {
			auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
			auto len2 = (*allDistances)[idJ_m][idI_p][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]
			auto nLen = len1 + len2;
			auto &len = pathPartShortest[idxPathPartShortest][neighID2]; // actual shortest path from start to [target][idx2]
			if (nLen < len.distance) {
				len.idClusterNode = neighID1;
				len.distance = nLen;
			}

		}
	}
	idxPathPartShortest++;

	double minLen = M;
	//INFO("calc shortest from "<< idI_p <<" to "<< idJ_p);
	for (int neighID1 = 0; neighID1 < samples_I_p.size(); ++neighID1) {
		for (int neighID2 = 0; neighID2 < samples_J_p.size(); ++neighID2) {
			auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
			auto len2 = (*allDistances)[idI_p][idJ_p][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]
			//INFO("shortest "<<" len1 "<<len1 <<" len2 "<<len2<<" normalOrderFromLen[idx2].second "<<normalOrderFromLen[idx2].second);
			auto nLen = len1 + len2 + normalOrderFromLen[neighID2].distance;
			if (nLen < minLen) {
				minLen = nLen;
			}
		}
	}
	//INFO("tryToExchange multi end");
	return minLen;
}

template<class T>
double VnsSopPath<T>::tryToMove(int idxWhatToMove, int idxWhereToMove) {
//move target index idxWhatToMove such that it has afterwards index idxWhereToMove
//INFO("tryToMove multi begin");
//INFO_GREEN("tryToMove idxWhatToMove " << idxWhatToMove << " idxWhereToMove " << idxWhereToMove);
//INFO("tryToMove getNumTargets "<<getNumTargets());
	const int TS = targets.size();
	if (idxWhatToMove >= TS || idxWhatToMove < 0 || idxWhereToMove > TS || idxWhereToMove < 0
			|| abs(idxWhereToMove - idxWhatToMove) < 2) {
		INFO_RED("tryToMove idxWhatToMove " << idxWhatToMove << " idxWhereToMove " << idxWhereToMove);
		INFO_RED("tryToMove getNumTargets "<<getNumTargets());
		INFO_RED(
				"tryToMove idxWhatToMove "<<idxWhatToMove<<" or idxWhereToMove "<<idxWhereToMove<<" exceeds maximum for target size"<<(TS));
		exit(1);
	}
//INFO("-------------------------------------------------------------------------");
	bool moveUp = idxWhereToMove > idxWhatToMove;
	int idxFrom = MIN(idxWhatToMove, idxWhereToMove);
	int idxTo = MAX(idxWhatToMove, idxWhereToMove);

	//std::stringstream ss;

	//change indexing from targets to samples
	int idxStartSamples = idxFrom + 1;
	int idxEndSamples = idxTo + 1;

	int nodeIdFrom = targets[idxFrom];
	int nodeIdto = targets[idxTo];

	int i = idxFrom + 1;

	int j = idxTo + 1;
	//INFO("get samples");
	std::vector<GraphNode<T>> samples_i_m = samples[i - 1];	//i-
	std::vector<GraphNode<T>> samples_i = samples[i];	//i
	std::vector<GraphNode<T>> samples_i_p = samples[i + 1];	//i+1
	std::vector<GraphNode<T>> samples_j_m = samples[j - 1];	//j-1
	std::vector<GraphNode<T>> samples_j = samples[j];	//j
	//INFO("get node ids");
	int idI_m = samples[i - 1][0].cluster_id;
	int idI = samples[i][0].cluster_id;
	int idI_p = samples[i + 1][0].cluster_id;

	int idJ_m = samples[j - 1][0].cluster_id;
	int idJ = samples[j][0].cluster_id;
	//INFO("get shortest");
	single_cluster_shortest_matrix normalOrderTotLen = shortest[i - 1];	// need to use idxStart+1 as to the

	shortest_matrix pathPartShortest;
	int numCityBetween = (idxEndSamples - idxStartSamples + 2);
	//INFO("numCityBetween "<<numCityBetween);
	pathPartShortest.resize(numCityBetween);

	pathPartShortest[0] = normalOrderTotLen;	// set actual shortest distance to idxStart
	double minLen = M;
	int idxPathPartShortest = 1;

	if (moveUp) {
		//move up
		//omit i and put the i between j_m and j
		//omit idxWhatToMove and put between idxWhereToMove-1 and idxWhereToMove
		pathPartShortest[idxPathPartShortest].assign(samples_i_p.size(), ClusterNodeDist(-1, M));

		//shortest after remove
		for (int neighID1 = 0; neighID1 < pathPartShortest[idxPathPartShortest - 1].size(); ++neighID1) {
			auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
			for (int neighID2 = 0; neighID2 < pathPartShortest[idxPathPartShortest].size(); ++neighID2) {
				auto len2 = (*allDistances)[idI_m][idI_p][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]
				auto nLen = len1 + len2;
				auto &len = pathPartShortest[idxPathPartShortest][neighID2]; // actual shortest path from start to [target][idx2]
				if (nLen < len.distance) {
					len.idClusterNode = neighID1;
					len.distance = nLen;
				}
			}
		}
		idxPathPartShortest++; //2
		//INFO("calc shortest from "<< idI_p <<" to other");
		for (int target = i + 1; target < j - 1; target++) {
			//INFO("target "<<target);
			int idFrom = target;
			int idTo = target + 1;
			pathPartShortest[idxPathPartShortest].assign(samples[idTo].size(), ClusterNodeDist(-1, M));

			for (int neighID1 = 0; neighID1 < samples[idFrom].size(); ++neighID1) {
				for (int neighID2 = 0; neighID2 < samples[idTo].size(); ++neighID2) {
					//			INFO("from");
					auto & from = samples[idFrom][neighID1];
					//		INFO("to");
					auto & to = samples[idTo][neighID2];
					//	INFO("len1");
					auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance;//shortest dist from start to [target - 1][idx1]
					//	INFO("len2");
					auto len2 = (*allDistances)[from.cluster_id][to.cluster_id][neighID1][neighID2];
					//	INFO("nLen");
					auto nLen = len1 + len2;
					auto &len = pathPartShortest[idxPathPartShortest][neighID2];// actual shortest path from start to [target][idx2]
					if (nLen < len.distance) {
						len.idClusterNode = neighID1;
						len.distance = nLen;
					}
				}
			}
			idxPathPartShortest++;
		}

		pathPartShortest[idxPathPartShortest].assign(samples_i.size(), ClusterNodeDist(-1, M));
		//INFO("b calc shortest from idJ_m "<< idJ_m <<" to idI "<< idI);
		for (int neighID1 = 0; neighID1 < pathPartShortest[idxPathPartShortest - 1].size(); ++neighID1) {
			for (int neighID2 = 0; neighID2 < pathPartShortest[idxPathPartShortest].size(); ++neighID2) {

				auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance;//shortest dist from start to [target - 1][idx1]
				//INFO("b get all distances");
				auto len2 = (*allDistances)[idJ_m][idI][neighID1][neighID2];//dist from [target - 1][idx1] to [target][idx2]
				auto nLen = len1 + len2;
				//INFO("b get pathPartShortest2");
				auto &len = pathPartShortest[idxPathPartShortest][neighID2];// actual shortest path from start to [target][idx2]
				if (nLen < len.distance) {
					len.idClusterNode = neighID1;
					len.distance = nLen;
				}

			}

		}
		idxPathPartShortest++;

		single_cluster_shortest_matrix normalOrderFromLen = shortest_back[j];
		//INFO("c calc shortest from idI "<< idI <<" to idJ "<< idJ);
		for (int neighID1 = 0; neighID1 < pathPartShortest[idxPathPartShortest - 1].size(); ++neighID1) {
			for (int neighID2 = 0; neighID2 < normalOrderFromLen.size(); ++neighID2) {
				auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
				auto len2 = (*allDistances)[idI][idJ][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]

				//INFO("shortest "<<" len1 "<<len1 <<" len2 "<<len2<<" normalOrderFromLen[idx2].second "<<normalOrderFromLen[idx2].second);
				auto nLen = len1 + len2 + normalOrderFromLen[neighID2].distance;
				if (nLen < minLen) {
					minLen = nLen;
				}
			}
		}

		//end move up
	} else {
		//INFO("move DOWN");
		//begin move down
		//move j between i-1 and i where j > i

		pathPartShortest[idxPathPartShortest].assign(samples_j.size(), ClusterNodeDist(-1, M));

		for (int neighID1 = 0; neighID1 < pathPartShortest[idxPathPartShortest - 1].size(); ++neighID1) {
			for (int neighID2 = 0; neighID2 < pathPartShortest[idxPathPartShortest].size(); ++neighID2) {
				auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance;//shortest dist from start to [target - 1][idx1]
				auto len2 = (*allDistances)[idI_m][idJ][neighID1][neighID2];//dist from [target - 1][idx1] to [target][idx2]
				auto nLen = len1 + len2;
				auto &len = pathPartShortest[idxPathPartShortest][neighID2];// actual shortest path from start to [target][idx2]
				if (nLen < len.distance) {
					len.idClusterNode = neighID1;
					len.distance = nLen;
				}
			}
		}
		idxPathPartShortest++; //2

		//INFO("e calc shortest from idJ "<< idJ <<" to idI "<< idI);
		pathPartShortest[idxPathPartShortest].assign(samples_i.size(), ClusterNodeDist(-1, M));
		for (int neighID1 = 0; neighID1 < pathPartShortest[idxPathPartShortest - 1].size(); ++neighID1) {
			for (int neighID2 = 0; neighID2 < pathPartShortest[idxPathPartShortest].size(); ++neighID2) {
				auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
				auto len2 = (*allDistances)[idJ][idI][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]
				auto nLen = len1 + len2;
				auto &len = pathPartShortest[idxPathPartShortest][neighID2]; // actual shortest path from start to [target][idx2]
				if (nLen < len.distance) {
					len.idClusterNode = neighID1;
					len.distance = nLen;
				}
			}
		}
		idxPathPartShortest++; //3

		for (int target = i; target < j - 1; target++) {
			//INFO("target "<<target);
			int idFrom = target;
			int idTo = target + 1;
			pathPartShortest[idxPathPartShortest].assign(samples[idTo].size(), ClusterNodeDist(-1, M));

			for (int neighID1 = 0; neighID1 < pathPartShortest[idxPathPartShortest - 1].size(); ++neighID1) {
				for (int neighID2 = 0; neighID2 < pathPartShortest[idxPathPartShortest].size(); ++neighID2) {
					auto & from = samples[idFrom][neighID1];
					auto & to = samples[idTo][neighID2];

					auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
					auto len2 = (*allDistances)[from.cluster_id][to.cluster_id][neighID1][neighID2];
					auto nLen = len1 + len2;
					auto &len = pathPartShortest[idxPathPartShortest][neighID2]; // actual shortest path from start to [target][idx2]

					if (nLen < len.distance) {
						len.idClusterNode = neighID1;
						len.distance = nLen;
					}

				}
			}

			idxPathPartShortest++;
		}

		int idJ_p = samples[j + 1][0].cluster_id;
		single_cluster_shortest_matrix normalOrderFromLen = shortest_back[j + 1];

		for (int neighID1 = 0; neighID1 < pathPartShortest[idxPathPartShortest - 1].size(); ++neighID1) {
			for (int neighID2 = 0; neighID2 < normalOrderFromLen.size(); ++neighID2) {
				auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
				auto len2 = (*allDistances)[idJ_m][idJ_p][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]
				auto nLen = len1 + len2 + normalOrderFromLen[neighID2].distance;
				if (nLen < minLen) {
					minLen = nLen;
				}
			}
		}
	}

	return minLen;
}

template<class T>
double VnsSopPath<T>::tryTwoOpt(int idxFrom, int idxTo) {
//INFO_GREEN("tryTwoOpt from "<<idxFrom <<" to "<<idxTo);

//exit when dist idxStart to idxEnd < 2 ---- need minimally distance of two*
//reverse path between idx1+1 and idx2, so need
//ensure idxStart <= idxEnd
	if (idxTo - idxFrom < 2) {
		INFO_RED("idxEnd and idxStart are too close "<<(idxTo - idxFrom)<<" must be lower than 2");
		exit(1);
	}
	if (idxTo >= targets.size()) {
		INFO_RED("idxEnd+1 is out of samples array "<<idxTo<<" samples number "<<targets.size());
		exit(1);
	}
	std::stringstream ss;
	//in targets
	//normal path   - till idxFrom-1
	//reversed path - from idxTo till idxFrom
	//normal path   - idxTo + 1

	//in samples
	//normal path   - till idxStartSamples-1
	//reversed path - from idxEndSamples till idxStartSamples
	//normal path   - idxEndSamples + 1

	//change indexing from targets to samples
	int idxStartSamples = idxFrom + 1;
	int idxEndSamples = idxTo + 1;
	//INFO("samples size "<<samples.size());
	//INFO("targets size "<<targets.size());
	//INFO("idxStartSamples "<<idxStartSamples);
	//INFO("idxEndSamples "<<idxEndSamples);

	int i = idxStartSamples - 1;
	int j = idxEndSamples;
	std::vector<GraphNode<T>> normalOrderToSamples = samples[i]; //i
	std::vector<GraphNode<T>> reverseOrderToSamples = samples[i + 1]; //i+1
	std::vector<GraphNode<T>> reverseOrderFromSamples = samples[j]; //j
	std::vector<GraphNode<T>> normalOrderFromSamples = samples[j + 1]; //j+1
	int idI = samples[i][0].cluster_id;
	int idI_p = samples[i + 1][0].cluster_id;
	int idJ = samples[j][0].cluster_id;
	int idJ_p = samples[j + 1][0].cluster_id;

	single_cluster_shortest_matrix normalOrderTotLen = shortest[i]; // need to use idxStart+1 as to the

	single_cluster_shortest_matrix normalOrderFromLen = shortest_back[j + 1];

	//INFO("find shortest paths between reverseOrderFromSamples to reverseOrderToSamples");
	// find shortest paths between reverseOrderFromSamples to reverseOrderToSamples
	// find the shortest part of path that is reversed
	// matrix:
	//    1) to city (layer)
	//    2) to idx i
	shortest_matrix pathPartShortest;
	int numCityBetween = (idxEndSamples - idxStartSamples + 2);
	//INFO("numCityBetween "<<numCityBetween);
	pathPartShortest.resize(numCityBetween);
	pathPartShortest[0] = normalOrderTotLen; // set actual shortest distance to idxStart

	int idxPathPartShortest = 1;
	//now calc shortest to reverseOrderFromSamples using distToReverseStart
	//INFO("calc shortest from "<<normalOrderToSamples[0].node.id <<" to "<< reverseOrderFromSamples[0].node.id);
	//pathPartShortest[idxPathPartShortest].assign(reverseOrderFromSamples.size(), dst_pair(-1, M));
	pathPartShortest[idxPathPartShortest].resize(reverseOrderFromSamples.size(), ClusterNodeDist(-1, M));

	for (int neighID1 = 0; neighID1 < pathPartShortest[idxPathPartShortest - 1].size(); ++neighID1) {
		for (int neighID2 = 0; neighID2 < pathPartShortest[idxPathPartShortest].size(); ++neighID2) {
			auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
			auto len2 = (*allDistances)[idI][idJ][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]
			auto nLen = len1 + len2;
			auto &len = pathPartShortest[idxPathPartShortest][neighID2]; // actual shortest path from start to [target][idx2]
			if (nLen < len.distance) {
				len.idClusterNode = neighID1;
				len.distance = nLen;
			}
		}
	}

	idxPathPartShortest++; //2

	//calc from j to i+1
	for (int target = idxEndSamples - 1; target >= idxStartSamples; target--) {

		pathPartShortest[idxPathPartShortest].assign(samples[target].size(), ClusterNodeDist(-1, M));

		int idFrom = target + 1;
		int idTo = target;
		//INFO("idFrom " << samples[idFrom][0].node.id);
		//INFO("idto "<< samples[idTo][0].node.id);

		for (int neighID1 = 0; neighID1 < samples[idFrom].size(); ++neighID1) {
			for (int neighID2 = 0; neighID2 < samples[idTo].size(); ++neighID2) {

				auto & from = samples[idFrom][neighID1];
				auto & to = samples[idTo][neighID2];

				auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
				auto len2 = (*allDistances)[from.cluster_id][to.cluster_id][neighID1][neighID2];

				auto nLen = len1 + len2;
				auto &len = pathPartShortest[idxPathPartShortest][neighID2]; // actual shortest path from start to [target][idx2]

				if (nLen < len.distance) {
					len.idClusterNode = neighID1;
					len.distance = nLen;
				}
			}
		}
		idxPathPartShortest++;
	}

	double minLen = M;
	for (int neighID1 = 0; neighID1 < pathPartShortest[idxPathPartShortest - 1].size(); ++neighID1) {
		for (int neighID2 = 0; neighID2 < normalOrderFromLen.size(); ++neighID2) {

			auto len1 = pathPartShortest[idxPathPartShortest - 1][neighID1].distance; //shortest dist from start to [target - 1][idx1]
			auto len2 = (*allDistances)[idI_p][idJ_p][neighID1][neighID2]; //dist from [target - 1][idx1] to [target][idx2]

			//INFO("shortest "<<" len1 "<<len1 <<" len2 "<<len2<<" normalOrderFromLen[idx2].second "<<normalOrderFromLen[idx2].second);
			auto nLen = len1 + len2 + normalOrderFromLen[neighID2].distance;
			if (nLen < minLen) {
				minLen = nLen;
			}
		}
	}
//INFO("minLen "<<minLen);
	return minLen;
}

template<class T>
void VnsSopPath<T>::twoOpt(int idxFrom, int idxTo) {

	int idxStartSamples = idxFrom + 1;
	int idxEndSamples = idxTo + 1;
//INFO("new two opt order is:")
//INFO("..... " <<" "<<idxStartSamples-1<<" "<<idxEndSamples<<" ..... "<<idxStartSamples <<" "<<idxEndSamples + 1<<" .....")
//exit when dist idxStart to idxEnd < 2 ---- need minimally distance of two*
// reverse path between idx1+1 and idx2, so need
	if (idxTo - idxFrom < 2) {
		INFO_RED("idxEnd and idxStart are too close "<<(idxTo - idxFrom)<<" must be lower than 2");
		exit(1);
	}
	if (idxTo >= targets.size()) {
		INFO_RED("idxTo is out of samples array "<<idxTo<<" samples number "<<targets.size());
		exit(1);
	}

	auto copy = targets;
	targets.clear();
//normal path   - till idxFrom-1
//reversed path - from idxTo till idxFrom
//normal path idxTo + 1
	for (int i = 0; i <= idxFrom - 1; i++) {
		targets.push_back(copy[i]);
	}
	for (int i = idxTo; i >= idxFrom; i--) {
		targets.push_back(copy[i]);
	}
	for (int i = idxTo + 1; i < copy.size(); i++) {
		targets.push_back(copy[i]);
	}

	update();
}

template<class T>
void VnsSopPath<T>::update() {
	//INFO("tour update");
	const int S = getSize();
	//INFO_VAR(S);
	// generate all samples
	//INFO("samples.resize(S + 1)"<<(S + 1));
	samples.resize(S + 1);
	int idx = 0;

	samples[0] = (*allSamples)[startClusterIdx];
	idx++;
	for (auto & t : targets) {
		samples[idx] = (*allSamples)[t];
		idx++;
	}
	samples[idx] = (*allSamples)[endClusterIdx];
	/*
	 //oldcalc of samples
	 generateSamples(samples[idx++], start, resolution, neighborhood_radius, neighborhood_resolution, null_start_goal_radius, start.id, end.id);
	 for (auto & t : targets) {
	 generateSamples(samples[idx++], t, resolution, neighborhood_radius, neighborhood_resolution, null_start_goal_radius, start.id, end.id);
	 }
	 generateSamples(samples[idx], end, resolution, neighborhood_radius, neighborhood_resolution, null_start_goal_radius, start.id, end.id);
	 */

	//INFO("samples generated");
	// find the shortest path - forward path
	//INFO("after update generate samples");
	shortest.resize(S + 1);
	shortest[0].assign(samples[0].size(), ClusterNodeDist());

	//INFO("shortest resized");
	for (int target = 1; target < S + 1; target++) {
		shortest[target].assign(samples[target].size(), ClusterNodeDist(-1, M));
		//INFO("target "<<target);
		//INFO_VAR(samples[target - 1].size())

		for (int fromClusterID = 0; fromClusterID < samples[target - 1].size(); ++fromClusterID) {
			auto & from = samples[target - 1][fromClusterID];
			//INFO_VAR(samples[target].size())
			for (int toNeighID = 0; toNeighID < samples[target].size(); ++toNeighID) {

				auto & to = samples[target][toNeighID];
				auto len1 = shortest[target - 1][fromClusterID].distance;
				//INFO_VAR(from.cluster_id)
				//INFO_VAR(to.cluster_id)
				auto len2 = (*allDistances)[from.cluster_id][to.cluster_id][fromClusterID][toNeighID];
				//INFO_VAR(len1);
				//INFO_VAR(len2);
				auto nLen = len1 + len2;

				auto &len = shortest[target][toNeighID];
				if (nLen < len.distance) {
					len.idClusterNode = fromClusterID;
					len.distance = nLen;
				}
			}
		}
	}

	// find the shortest path - backward path
	shortest_back.resize(S + 1);
	shortest_back[S].assign(samples[S].size(), ClusterNodeDist());
	//INFO("shortest_back resized");

	for (int target = S - 1; target >= 0; target--) {
		shortest_back[target].assign(samples[target].size(), ClusterNodeDist(-1, M));
		for (int fromClusterID = 0; fromClusterID < samples[target].size(); ++fromClusterID) {
			auto & from = samples[target][fromClusterID];
			for (int toNeighID = 0; toNeighID < samples[target + 1].size(); ++toNeighID) {
				auto & to = samples[target + 1][toNeighID];
				auto len1 = shortest_back[target + 1][toNeighID].distance;
				auto len2 = (*allDistances)[from.cluster_id][to.cluster_id][fromClusterID][toNeighID];
				//INFO_VAR(len1);
				//INFO_VAR(len2);
				auto nLen = len1 + len2;
				auto &len = shortest_back[target][fromClusterID];
				if (nLen < len.distance) {
					len.idClusterNode = toNeighID;
					len.distance = nLen;
				}
			}
		}

	}

	//INFO("update end");
	//print(*allDistances)();
	//printShortestDistances();
}

template<class T>
void VnsSopPath<T>::updateAfterInsert(int idxStart, int idxEnd) {
	//INFO(" ------------------- ");
	//INFO("updateAfterInsert "<<idxStart<<" "<<idxEnd);
	const int S = getSize();	//number of dubins maneuvers

	//INFO("generateSamples");
	std::vector<std::vector<GraphNode<T>>> insertedSamples;
	insertedSamples.resize(idxEnd - idxStart + 1);
	for (int var = idxStart; var <= idxEnd; ++var) {
		insertedSamples[var - idxStart] = (*allSamples)[targets[var]];
	}

	//INFO("ids before:");
	//this->listIds();
	samples.insert(samples.begin() + idxStart + 1, insertedSamples.begin(), insertedSamples.end());
	//INFO("ids after:");
	//this->listIds();
	//INFO("shortest insert ");
	shortest.insert(shortest.begin() + idxStart + 1, idxEnd - idxStart + 1, single_cluster_shortest_matrix());

	for (int target = idxStart + 1; target < S + 1; target++) {
		//INFO("resize shortes to "<<samples[target].size());

		shortest[target].assign(samples[target].size(), ClusterNodeDist(-1, M));

		//INFO("get dist from "<<samples[target - 1][0][0].node.id);
		//INFO("get dist to "<<samples[target ][0][0].node.id);

		for (int fromNeighID = 0; fromNeighID < samples[target - 1].size(); ++fromNeighID) {
			auto & from = samples[target - 1][fromNeighID];
			for (int toNeighID = 0; toNeighID < samples[target].size(); ++toNeighID) {
				auto & to = samples[target][toNeighID];
				auto len1 = shortest[target - 1][fromNeighID].distance;
				auto len2 = (*allDistances)[from.cluster_id][to.cluster_id][fromNeighID][toNeighID];
				auto nLen = len1 + len2;
				auto &len = shortest[target][toNeighID];
				if (nLen < len.distance) {
					len.idClusterNode = fromNeighID;
					len.distance = nLen;
				}
			}
		}
	}

	//INFO("shortest_back insert ");
	shortest_back.insert(shortest_back.begin() + idxStart + 1, idxEnd - idxStart + 1, single_cluster_shortest_matrix());

	for (int target = idxEnd + 1; target >= 0; target--) {
		shortest_back[target].assign(samples[target].size(), ClusterNodeDist(-1, M));

		for (int fromNeighID = 0; fromNeighID < samples[target].size(); ++fromNeighID) {
			auto & from = samples[target][fromNeighID];

			for (int toNeighID = 0; toNeighID < samples[target + 1].size(); ++toNeighID) {
				auto & to = samples[target + 1][toNeighID];
				auto len1 = shortest_back[target + 1][toNeighID].distance;
				auto len2 = (*allDistances)[from.cluster_id][to.cluster_id][fromNeighID][toNeighID];

				auto nLen = len1 + len2;
				auto &len = shortest_back[target][fromNeighID];
				if (nLen < len.distance) {
					len.idClusterNode = toNeighID;
					len.distance = nLen;
				}
			}
		}
	}

//INFO("updateAfterInsert end");
//INFO("shortest_back evaluated ");
}

template<class T>
void VnsSopPath<T>::updateAfterRemove(int idxStart, int idxEnd) {
//INFO("updateAfterRemove "<<idxStart<<" "<<idxEnd);
	const int S = getSize();	//number of dubins maneuvers
//INFO("samples erase ");
	samples.erase(samples.begin() + idxStart + 1, samples.begin() + idxEnd + 2);

//INFO("shortest erase ");
	shortest.erase(shortest.begin() + idxStart + 1, shortest.begin() + idxEnd + 2);
//INFO("recalc shortest");

	for (int target = idxStart + 1; target < S + 1; target++) {
//INFO("----------------");
		shortest[target].assign(samples[target].size(), ClusterNodeDist(-1, M));

		for (int fromNeighID = 0; fromNeighID < samples[target - 1].size(); ++fromNeighID) {
			auto & from = samples[target - 1][fromNeighID];
			const int fromID = from.cluster_id;

			for (int toNeighID = 0; toNeighID < samples[target].size(); ++toNeighID) {
				auto & to = samples[target][toNeighID];
				const int toID = to.cluster_id;
				auto len1 = shortest[target - 1][fromNeighID].distance;
				//auto len2 = distances[target - 1][idx1][idx2];
				auto len2 = (*allDistances)[fromID][toID][fromNeighID][toNeighID];
				auto nLen = len1 + len2;
				auto &len = shortest[target][toNeighID];
				if (nLen < len.distance) {
					len.idClusterNode = fromNeighID;
					len.distance = nLen;
				}

			}
		}
	}

	shortest_back.erase(shortest_back.begin() + idxStart + 1, shortest_back.begin() + idxEnd + 2);

	for (int target = idxStart; target >= 0; target--) {
		shortest_back[target].assign(samples[target].size(), ClusterNodeDist(-1, M));

		for (int fromNeighID = 0; fromNeighID < samples[target].size(); ++fromNeighID) {
			auto & from = samples[target][fromNeighID];

			const int fromID = from.cluster_id;

			for (int toNeighID = 0; toNeighID < samples[target + 1].size(); ++toNeighID) {
				auto & to = samples[target + 1][toNeighID];
				const int toID = to.cluster_id;
				auto len1 = shortest_back[target + 1][toNeighID].distance;
				//auto len2 = distances[target][idx1][idx2];
				auto len2 = (*allDistances)[fromID][toID][fromNeighID][toNeighID];
				auto nLen = len1 + len2;
				//INFO("len1 "<<len1<<" len2 "<<len2);
				auto &len = shortest_back[target][fromNeighID];
				if (nLen < len.distance) {
					len.idClusterNode = toNeighID;
					len.distance = nLen;
				}
			}
		}

	}
}

template<class T>
std::vector<IndexSOP> VnsSopPath<T>::getPath() const {
	//INFO("getPath begin");
	std::vector<IndexSOP> ret;
	const int S = getSize();

	int target = S;
	int inClusterID = 0;
	double minLen = M;
	//INFO("chainGeneratingNeighAngEnd");
	if (!shortest.empty()) {

		for (int i = 0; i < shortest[S].size(); i++) {
			auto & p = shortest[S][i];
			if (p.distance < minLen) {
				minLen = p.distance;
				inClusterID = i;
			}
		}

		//INFO("getPath start "<<target<<" "<<inClusterID);

		//std::vector<double> neigh_angs;
		//neigh_angs.push_back(samples[target][neighID][idx].neigh_ang);

		IndexSOP newInPathId;
		newInPathId.clusterIndex = samples[target][inClusterID].cluster_id;
		newInPathId.nodeIndex = inClusterID;
		ret.push_back(newInPathId);
		//INFO("while")
		while (target > 0) {
			std::vector<GraphNode<T>> fromStates = samples[target - 1];
			std::vector<GraphNode<T>> toSamples = samples[target];

			int lastinClusterID = shortest[target][inClusterID].idClusterNode;

			GraphNode<T> & from = fromStates[lastinClusterID];
			//neigh_angs.push_back(from.neigh_ang);
			GraphNode<T> & to = toSamples[inClusterID];

			IndexSOP newInPathId;
			newInPathId.clusterIndex = from.cluster_id;
			newInPathId.nodeIndex = lastinClusterID;
			ret.push_back(newInPathId);

			inClusterID = lastinClusterID;
			target--;
		}

		//INFO("reverse")
		std::reverse(ret.begin(), ret.end());
	}
	//INFO("getPath end");
	return ret;
}

template<class T>
std::vector<GraphNode<T>> VnsSopPath<T>::getPathNeighAngIds(GraphNode<T> & chainGeneratingNeighAngEnd,
		bool fillchainGenNA) const {
//INFO("getPathNeighAngIds begin");
//INFO("chainGeneratingNeighAngFrom " << chainGeneratingNeighAngEnd.idNeigh << " " << chainGeneratingNeighAngEnd.idAng);
	const int S = getSize();
	std::vector<GraphNode<T>> ret;
	ret.reserve(S + 1);

	int target = S;
	int neighID = 0;
	double minLen = M;

	double sum = 0;

	if (chainGeneratingNeighAngEnd.cluster_id == -1) {
//not specified where to get the generating path
		for (int i = 0; i < shortest[S].size(); i++) {

			auto & p = shortest[S][i];
			if (p.distance < minLen) {
				minLen = p.distance;
				neighID = i;
			}
		}
		if (fillchainGenNA) {
			chainGeneratingNeighAngEnd.cluster_id = neighID;
		}
	} else {

		neighID = chainGeneratingNeighAngEnd.cluster_id;

	}

	GraphNode<T> neighAngId;
	neighAngId = samples[S][neighID];
	ret.push_back(neighAngId);
//INFO("first added "<<S);

	while (target > 0) {
		std::vector<GraphNode<T>> fromStates = samples[target - 1];
		std::vector<GraphNode<T>> toSamples = samples[target];

		int lastNeighID = shortest[target][neighID].idClusterNode;

		neighID = lastNeighID;

		GraphNode<T> neighAngId;
		neighAngId = samples[target - 1][neighID];
		ret.push_back(neighAngId);

		//INFO("other added "<<target-1);
		target--;
	}

	std::reverse(ret.begin(), ret.end());
	//INFO("getPathNeighAngIds end");
	return ret;
}

template<class T>
double VnsSopPath<T>::getPathLength(const GraphNode<T> & chainGeneratingNeighAngEnd) const {
//INFO("getPathLength beg");
	const int S = getSize();
	//INFO("shortest.size() "<<shortest.size());
	double minLen = M;
	if (chainGeneratingNeighAngEnd.cluster_id == -1) {
		for (int neigh = 0; neigh < shortest[S].size(); neigh++) {
			auto p = shortest[S][neigh];
			if (p.distance < minLen) {
				minLen = p.distance;
			}
		}
	} else {
		int neigh = chainGeneratingNeighAngEnd.cluster_id;
		minLen = shortest[S][neigh].distance;
	}
	//INFO("getPathLength end");
	return minLen;
}

template<class T>
double VnsSopPath<T>::getPathLengthRear(const GraphNode<T> & chainGeneratingNeighAngStart) const {
	//INFO("getPathLength beg");
	//INFO("shortest.size() "<<shortest.size());
	double minLen = M;
	if (chainGeneratingNeighAngStart.cluster_id == -1) {
		for (int neigh = 0; neigh < shortest_back[0].size(); neigh++) {
			auto p = shortest_back[0][neigh];
			if (p.distance < minLen) {
				minLen = p.distance;
			}

		}
	} else {
		int neigh = chainGeneratingNeighAngStart.cluster_id;
		minLen = shortest_back[0][neigh].distance;
	}
	//INFO("getPathLength end");
	return minLen;
}

template<class T>
double VnsSopPath<T>::getPathLengthCalculate(const GraphNode<T> & chainGeneratingNeighAngEnd) const {
//INFO("getPathLengthCalculate begin");

	const int S = getSize();

	int target = S;
	int neighID = 0;
	double minLen = M;

	double total_length_sum = 0;
	if (chainGeneratingNeighAngEnd.cluster_id == -1) {
		for (int neigh = 0; neigh < shortest[S].size(); neigh++) {

			auto p = shortest[S][neigh];
			if (p.distance < minLen) {
				minLen = p.distance;
				neighID = neigh;

			}

		}
	} else {
		neighID = chainGeneratingNeighAngEnd.cluster_id;
	}
	while (target > 0) {
		std::vector<GraphNode<T>> fromStates = samples[target - 1];
		std::vector<GraphNode<T>> toSamples = samples[target];

		int lastNeighID = shortest[target][neighID].idClusterNode;

		auto & from = fromStates[lastNeighID];

		auto & to = toSamples[neighID];

		total_length_sum += (*allDistances)[from.cluster_id][to.cluster_id][lastNeighID][neighID];

		neighID = lastNeighID;
		target--;
	}

	return total_length_sum;
}

template<class T>
double VnsSopPath<T>::getPathLengthCalculateRear(const GraphNode<T> & chainGeneratingNeighAngStart) const {

	int neighID = 0;
	double minLen = M;
	double total_length_sum = 0;
	if (chainGeneratingNeighAngStart.cluster_id == -1) {
		for (int neigh = 0; neigh < shortest_back[0].size(); neigh++) {
			auto p = shortest_back[0][neigh];
			if (p.distance < minLen) {
				minLen = p.distance;
				neighID = neigh;
			}
		}
	} else {
		int neigh = chainGeneratingNeighAngStart.cluster_id;
		minLen = shortest_back[0][neigh].distance;
	}

	for (int target = 1; target < shortest_back.size(); ++target) {
		std::vector<GraphNode<T>> fromStates = samples[target - 1];
		std::vector<GraphNode<T>> toSamples = samples[target];

		int nextNeighID = shortest_back[target - 1][neighID].idClusterNode;

		auto & from = fromStates[neighID];

		auto & to = toSamples[nextNeighID];

		total_length_sum += (*allDistances)[from.cluster_id][to.cluster_id][neighID][nextNeighID];

		neighID = nextNeighID;
	}

	return total_length_sum;
}

template<class T>
int VnsSopPath<T>::getNumTargets() {
	return targets.size();
}

template<class T>
int VnsSopPath<T>::getTarget(int idx) {
	if (idx >= targets.size() || idx < 0) {
		INFO_RED("getTarget idx "<<idx<<" exceeds maximum "<<(targets.size()-1));
		exit(1);
	}
	return targets[idx];
}

template<class T>
std::vector<int> VnsSopPath<T>::getTargets(int idxFrom, int idxTo) {
	return std::vector<int>(targets.begin() + idxFrom, targets.begin() + idxTo + 1);
}

template<class T>
std::vector<std::vector<GraphNode<T>>> VnsSopPath<T>::getSamples() {
	return samples;
}

template<class T>
std::vector<std::vector<GraphNode<T>>> &VnsSopPath<T>::getSamplesRef() {
	return samples;
}

template<class T>
shortest_matrix & VnsSopPath<T>::getShortestRef() {
	return shortest;
}

template<class T>
shortest_matrix & VnsSopPath<T>::getShortestBackRef() {
	return shortest_back;
}

template<class T>
int VnsSopPath<T>::getNumSamples() {
	return samples.size();
}

template<class T>
void VnsSopPath<T>::checkShortestConsistency() {
//get shortest in the end
	INFO("checkShortestConsistency begin");

	const int S = getSize();
	double minLenEnd = M;
	int neighIdEnd = 0;
	int headIdEnd = 0;
	for (int neigh = 0; neigh < shortest[S].size(); neigh++) {
		auto p = shortest[S][neigh];
		if (p.distance < minLenEnd) {
			minLenEnd = p.distance;
			neighIdEnd = neigh;
		}

	}

	int neighIdEnd_front = neighIdEnd;
	int headIdEnd_front = headIdEnd;
	for (int target = S; target > 0; --target) {
		int nextIdNeigh = shortest[target][neighIdEnd_front].idClusterNode;
		int nextIdHead = shortest[target][neighIdEnd_front].idClusterNode;

		neighIdEnd_front = nextIdNeigh;
		headIdEnd_front = nextIdHead;
	}

//get shortest in the front
	double minLenFront = M;
	int neighIdFront = 0;
	int headIdFront = 0;
	for (int neigh = 0; neigh < shortest_back[0].size(); neigh++) {
		auto p = shortest_back[0][neigh];
		if (p.distance < minLenFront) {
			minLenFront = p.distance;
			neighIdFront = neigh;
		}

	}

	int neighIdFront_end = neighIdFront;
	int headIdFront_end = headIdFront;
	for (int target = 0; target < S; ++target) {
		int nextIdNeigh = shortest_back[target][neighIdFront_end].idClusterNode;

		neighIdFront_end = nextIdNeigh;
	}

	INFO_VAR(minLenEnd);
	INFO_VAR(minLenFront);
	INFO("end shortest "<<neighIdEnd<<" "<<headIdEnd);
	INFO("front shortest "<<neighIdEnd_front<<" "<<headIdEnd_front);
	INFO("end shortest_back "<<neighIdFront_end<<" "<<headIdFront_end);
	INFO("front shortest_back "<<neighIdFront<<" "<<headIdFront);

	INFO("checkShortestConsistency end");
}

template<class T>
void VnsSopPath<T>::checkSameSamples(std::string text) {
//INFO("check_same_samples");

	if (samples[0].size() == (*allSamples)[startClusterIdx].size()) {
		for (int neigh = 0; neigh < samples[0].size(); ++neigh) {

			if (samples[0][neigh].cluster_id != (*allSamples)[startClusterIdx][neigh].cluster_id
					|| fabs(samples[0][neigh].x - (*allSamples)[startClusterIdx][neigh].x) > 0.01
					|| fabs(samples[0][neigh].y - (*allSamples)[startClusterIdx][neigh].y) > 0.01) {
				INFO("not same value start "<<text);
				INFO_VAR(neigh);
				INFO_VAR(samples[0][neigh].cluster_id);
				INFO_VAR((*allSamples)[startClusterIdx][neigh].cluster_id);
				INFO_VAR(samples[0][neigh].x);
				INFO_VAR((*allSamples)[startClusterIdx][neigh].x);
				INFO_VAR(samples[0][neigh].y);
				INFO_VAR((*allSamples)[startClusterIdx][neigh].y);
				exit(1);
			}

			else {
				INFO("not same size start neigh");
				exit(1);
			}
		}
	}
	int idx = 1;
	for (auto & t : targets) {
		if (samples[idx].size() == (*allSamples)[t].size()) {
			for (int neigh = 0; neigh < (*allSamples)[t].size(); ++neigh) {

				if (samples[idx][neigh].cluster_id != (*allSamples)[t][neigh].cluster_id
						|| fabs(samples[idx][neigh].x - (*allSamples)[t][neigh].x) > 0.01
						|| fabs(samples[idx][neigh].y - (*allSamples)[t][neigh].y) > 0.01) {
					INFO("not same value idx "<<idx<<" "<<text);
					INFO_VAR(neigh);
					INFO_VAR(samples[idx][neigh].cluster_id);
					INFO_VAR((*allSamples)[t][neigh].cluster_id);
					INFO_VAR(samples[idx][neigh].x);
					INFO_VAR((*allSamples)[t][neigh].x);
					INFO_VAR(samples[idx][neigh].y);
					INFO_VAR((*allSamples)[t][neigh].y);
					exit(1);
				}

			}
		} else {
			INFO("not same size idx "<<idx);
			exit(1);
		}
		idx++;

	}

	if (samples[idx].size() == (*allSamples)[endClusterIdx].size()) {
		for (int neigh = 0; neigh < (*allSamples)[endClusterIdx].size(); ++neigh) {

			if (samples[idx][neigh].cluster_id != (*allSamples)[endClusterIdx][neigh].cluster_id
					|| fabs(samples[idx][neigh].x - (*allSamples)[endClusterIdx][neigh].x) > 0.01
					|| fabs(samples[idx][neigh].y - (*allSamples)[endClusterIdx][neigh].y) > 0.01) {
				INFO("not same value end "<<text);
				INFO_VAR(neigh);
				INFO_VAR(samples[idx][neigh].cluster_id);
				INFO_VAR((*allSamples)[endClusterIdx][neigh].cluster_id);
				INFO_VAR(samples[idx][neigh].x);
				INFO_VAR((*allSamples)[endClusterIdx][neigh].x);
				INFO_VAR(samples[idx][neigh].y);
				INFO_VAR((*allSamples)[endClusterIdx][neigh].y);
				exit(1);
			}

		}
	} else {
		INFO("not same size end");
		exit(1);
	}

}

template<class T>
void VnsSopPath<T>::listIds() {
	std::cout << "cluster ids: " << std::endl;
	for (int var = 0; var < samples.size(); ++var) {
		std::cout << samples[var][0].cluster_id << " ";
	}
	std::cout << std::endl;
}

template<class T>
void VnsSopPath<T>::listRewads() {
	std::cout << "target rewards: " << std::endl;
	for (int var = 0; var < samples.size(); ++var) {
		std::cout << samples[var][0].reward << " ";
	}
	std::cout << std::endl;
}

template<class T>
void VnsSopPath<T>::listNodeIds() {
	GraphNode<T> dummy = GraphNode<T>();
	std::vector<GraphNode<T>> samplesNeighAngs = this->getPathNeighAngIds(dummy);
	std::cout << "node ids: " << std::endl;
	for (int var = 0; var < samplesNeighAngs.size(); ++var) {
		std::cout << samplesNeighAngs[var].id << " ";
	}
	std::cout << std::endl;
	std::cout << "cluster ids for nodes: " << std::endl;
	for (int var = 0; var < samplesNeighAngs.size(); ++var) {
		std::cout << samplesNeighAngs[var].cluster_id << " ";
	}
	std::cout << std::endl;
}

template<class T>
void VnsSopPath<T>::printShortestDistances() {
	std::cout << "shortest: " << std::endl;
	for (int var = 0; var < shortest.size(); ++var) {
		for (int var2 = 0; var2 < shortest[var].size(); ++var2) {
			std::cout << shortest[var][var2].distance << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
	std::cout << "shortest_back: " << std::endl;
	for (int var = 0; var < shortest_back.size(); ++var) {
		for (int var2 = 0; var2 < shortest_back[var].size(); ++var2) {
			std::cout << shortest_back[var][var2].distance << " ";
		}
		std::cout << std::endl;
	}

}

template<class T>
std::vector<int>::iterator VnsSopPath<T>::targetsBegin() {
	return targets.begin();
}

template<class T>
std::vector<int>::iterator VnsSopPath<T>::targetsEnd() {
	return targets.end();
}

template<class T>
double VnsSopPath<T>::getReward() {
	double reward = 0;
	for (int var = 0; var < targets.size(); ++var) {
		reward += (*allSamples)[targets[var]][0].reward;
	}
	if ((*allSamples)[startClusterIdx][0].reward > 0) {
		reward += (*allSamples)[startClusterIdx][0].reward;
		//INFO("add start reward");
		if ((*allSamples)[startClusterIdx][0].cluster_id != (*allSamples)[endClusterIdx][0].cluster_id) {
			//INFO("add end reward");
			reward += (*allSamples)[endClusterIdx][0].reward;
		}
	}

	return reward;
}

template<class T>
std::vector<T> VnsSopPath<T>::getPathSampled(double samplePathDistance) {
	//T = HeapPoint2D
	//T = HeapPoint2DHeading
	std::vector<IndexSOP> returnedPath = this->getPath();
	std::vector<T> path;

	bool allSampled = false;
	int solutionIndexTo = 1;
	//double distancle_left = 0;
	std::vector<HeapNode<T>> plan_all;

	if (returnedPath.size() >= 2) {
		std::vector<HeapNode<T>> plan_all;
		for (int var = 1; var < returnedPath.size(); ++var) {
			int node_id_from = (*allSamples)[returnedPath[var - 1].clusterIndex][returnedPath[var - 1].nodeIndex].id;
			int node_id_to = (*allSamples)[returnedPath[var].clusterIndex][returnedPath[var].nodeIndex].id;
			plan_with_length<T> found_path = prm->plan(node_id_from, node_id_to);
			if (var == 1) {
				plan_all.insert(std::end(plan_all), std::begin(found_path.plan), std::end(found_path.plan));
			} else {
				plan_all.insert(std::end(plan_all), std::begin(found_path.plan) + 1, std::end(found_path.plan));
			}
		}

		for (int var = 1; var < plan_all.size(); ++var) {
			double d = plan_all[var - 1].data.distance(plan_all[var].data);
			if (d < 0.001) {
				INFO("distance is bellow 0.001 "<<d)
			}
		}

		INFO("samplePathDistance "<<samplePathDistance)

		path.push_back(plan_all[0].data);
		T node_from = plan_all[0].data;
		while (!allSampled) {
			if (solutionIndexTo < plan_all.size()) {
				T node_from = path[path.size()-1];
				T node_to = plan_all[solutionIndexTo].data;

				double actualLen = node_from.distance(node_to);
				//INFO("actualLen "<<actualLen<<" sit "<<solutionIndexTo)

				if (actualLen > samplePathDistance ) {
					T addState = node_from.getStateInDistance(node_to, samplePathDistance);
					path.push_back(addState);
					//distancle_left = 0;
					//INFO("distancle_left "<<distancle_left)
					if (path.size() >= 2) {
						double dist_path_added = path[path.size() - 2].distance(path[path.size() - 1]);
						//INFO("dist_added "<<dist_path_added)
					}
				} else {
					//distancle_left = samplePathDistance - actualLen;
					//INFO("distancle_left "<<distancle_left)
					solutionIndexTo++;
				}
			} else {
				T lastNode = plan_all[plan_all.size() - 1].data;
				path.push_back(lastNode);
				allSampled = true;
			}
		}
	}

	return path;
}

#endif

