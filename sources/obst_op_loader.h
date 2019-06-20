/*
 * GOPLoader.h
 *
 *  Created on: Dec 11, 2017
 *      Author: penicrob
 */

#ifndef OBST_OP_LOADER_H_
#define OBST_OP_LOADER_H_

#include "dataset_loader_obst.h"
#include "heuristic_types.h"
#include "math_common.h"
#include "mesh_object.h"
extern "C" {
#include "triangle/triangle.h"
}

using namespace crl;

class OBSTOPLoader {
public:
	OBSTOPLoader();
	virtual ~OBSTOPLoader();

	static CoordsVector getCoordPath(std::vector<IndexSOP>& idPaths, samples_type_2d & nodesAllClusters, GOPTYPE gopType, CConfig& config);

	static OP_Prolem<HeapPoint3D> getSOPDefinition(CConfig& config, const std::string& problemFile, bool change_end_index_to_last = false);
	static OP_Prolem<HeapPoint3D> getSOPDefinitionMapFile(CConfig& config, const std::string& problemFile, bool change_end_index_to_last = false);
	static void fixClusterNodeRenumbering(std::vector<GraphNode<HeapPoint2D>>& finalPath, int startIndex, int goalIndex, int oldstartIndex, int oldgoalIndex,
			int deleted_cluster_id, bool numbered_from_one);


	//for classical OP
	static OP_Prolem<HeapPoint2D> getOP(std::vector<GraphNode<HeapPoint2D>> opNodes);
	static CoordsVector getOBSTOPPath(std::vector<IndexSOP> &gopPath, samples_type_2d & nodesAllClusters);

	static std::vector<MeshObject*> create_mesh_obstacles(std::vector<MapPoint> points_vec, std::vector<Obstacle> obstacles_vec);

	static void printAllClustersNodes(samples_type_2d samples);
};

#endif /* OBST_OP_LOADER_H_ */
