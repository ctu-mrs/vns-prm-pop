/*
 * GOPLoader.cpp
 *
 *  Created on: Dec 11, 2017
 *      Author: penicrob
 */

#include "obst_op_loader.h"
using crl::logger;
using namespace crl;

#define ROBOT_NAME "robot"
#define START_NAME "start"
#define GOAL_NAME "goal"
#define TARGET_NAME_START "target."
#define BORDER_NAME "border"

OBSTOPLoader::OBSTOPLoader() {
}

OBSTOPLoader::~OBSTOPLoader() {
}

CoordsVector OBSTOPLoader::getCoordPath(std::vector<IndexSOP>& idPaths, samples_type_2d & nodesAllClusters,
		GOPTYPE gopType, CConfig& config) {
	CoordsVector coords;
	//INFO("getCoordPath begin");
	switch (gopType) {
	case OBST_OP: {
		coords = getOBSTOPPath(idPaths, nodesAllClusters);
		break;
	}
	default: {
		INFO("draw path not implemented for gop type "<<gopType);
		break;
	}
	}
	//INFO("getCoordPath end");
	return coords;
}

OP_Prolem<HeapPoint3D> OBSTOPLoader::getSOPDefinition(CConfig& config, const std::string& problemFile,
		bool change_end_index_to_last) {
	OP_Prolem<HeapPoint3D> problem;
	std::string gop_type = config.get<std::string>("gop-type");
	gop_type = trim(gop_type);
	INFO("compare gop-type:"<<gop_type);

	if (gop_type.compare("obst_op") == 0) {
		INFO("gop type is sop");
		problem.gop_type = SOP;
		DatasetOBST_OP<HeapPoint3D> loadedDataset = DatasetLoaderOBSTOP::loadDatasetMapCityPoints(problemFile);
		INFO("has loadedDataset")
		INFO("use budget " << loadedDataset.Tmax);
		INFO("use startIndex " << loadedDataset.startID);
		INFO("use goalIndex " << loadedDataset.goalID);
		problem.name = config.get<std::string>("name");
		problem.startIndex = loadedDataset.startID;
		problem.goalIndex = loadedDataset.goalID;
		problem.name = loadedDataset.name;
		problem.budget = loadedDataset.Tmax;
		problem.border = loadedDataset.border;
		problem.obstacles = loadedDataset.obstacles;
		problem.map_points = loadedDataset.map_points;
		problem.convex_regions = loadedDataset.convex_regions;
		problem.city2map_visibility_lists = loadedDataset.city2map_visibility_lists;
		problem.map_visibility_lists = loadedDataset.map_visibility_lists;
		problem.city2city_visibility_lists = loadedDataset.city2city_visibility_lists;

		//DatasetLoaderSOP::printSets(loadedDataset);
		std::vector<std::vector<GraphNode<HeapPoint3D>>> clusterNodes;
		for (int var = 0; var < loadedDataset.graph.size(); ++var) {
			std::vector<GraphNode<HeapPoint3D>> singleNodeCluster;
			GraphNode<HeapPoint3D> gn = loadedDataset.graph[var];
			gn.cluster_id = var;
			singleNodeCluster.push_back(gn);
			clusterNodes.push_back(singleNodeCluster);
		}
		INFO("load dataset problem.startIndex "<<problem.startIndex)
		INFO("load dataset problem.goalIndex "<<problem.goalIndex)
		problem.samples = clusterNodes;

		//create obstacels from borders
		for (int var = 0; var < problem.border.size(); ++var) {
			//jde to proti hodinovym rucickam
			int prev_prev_point = problem.border[(var - 2) % problem.border.size()];
			int prev_point = problem.border[(var - 1) % problem.border.size()];
			int act_point = problem.border[var];
			int next_point = problem.border[(var + 1) % problem.border.size()];
			//INFO("prev_prev_point "<<prev_prev_point <<" prev_point "<<prev_point<<" act_point "<<act_point<<" next_point "<<next_point)
			Point mp_act = problem.map_points[act_point].toPoint();
			Point mp_prev_prev = problem.map_points[prev_prev_point].toPoint();
			Point mp_prev = problem.map_points[prev_point].toPoint();
			Point mp_next = problem.map_points[next_point].toPoint();

			int new_id = problem.map_points.size();

			double border_distance = mp_act.distance(mp_prev);
			Vector vec_outside_prev = (mp_prev - mp_prev_prev).normalize() * 100;
			Point new_point_prev = mp_prev + vec_outside_prev;
			Vector vec_outside1 = (mp_act - mp_prev).normalize() * 100;
			Point new_point_act_cont = mp_act + vec_outside1;
			Vector vec_outside2 = (mp_act - mp_next).normalize() * 100;
			Point new_point_act_side = new_point_act_cont + vec_outside2;

			MapPoint new_map_point_act_cont;
			new_map_point_act_cont.id = new_id;
			new_map_point_act_cont.x = new_point_act_cont.x;
			new_map_point_act_cont.y = new_point_act_cont.y;
			INFO("new_map_point_act_cont "<<new_map_point_act_cont)
			problem.map_points.push_back(new_map_point_act_cont);

			MapPoint new_map_point_act_side;
			new_map_point_act_side.id = new_id + 1;
			new_map_point_act_side.x = new_point_act_side.x;
			new_map_point_act_side.y = new_point_act_side.y;
			INFO("new_map_point_act_side "<<new_map_point_act_side)
			problem.map_points.push_back(new_map_point_act_side);

			MapPoint new_map_point_prew;
			new_map_point_prew.id = new_id + 2;
			new_map_point_prew.x = new_point_prev.x;
			new_map_point_prew.y = new_point_prev.y;
			INFO("new_map_point_prew "<<new_map_point_prew)
			problem.map_points.push_back(new_map_point_prew);

			Obstacle border_obstacle;
			border_obstacle.point_indexes.push_back(prev_point);
			border_obstacle.point_indexes.push_back(new_id);
			border_obstacle.point_indexes.push_back(new_id + 1);
			border_obstacle.point_indexes.push_back(new_id + 2);
			problem.obstacles.push_back(border_obstacle);

		}
		problem.mesh_obstacles = create_mesh_obstacles(problem.map_points, problem.obstacles);

		problem.mesh_robot = new MeshObject;
		problem.mesh_robot->BeginObject();

		double robot_size = config.get<double>("robot-size");
		INFO("using robot size "<<robot_size);
		if(robot_size==0){
			ERROR("you need to specify robot size for this 2d problem");
			exit(1);
		}

		problem.mesh_robot->addVertex(-robot_size, -robot_size, 0);
		problem.mesh_robot->addVertex(robot_size, -robot_size, 0);
		problem.mesh_robot->addVertex(robot_size, robot_size, 0);
		problem.mesh_robot->addVertex(-robot_size, robot_size, 0);
		problem.mesh_robot->addFace(0, 1, 2);
		problem.mesh_robot->addFace(0, 2, 3);

		/*
		problem.mesh_robot->addVertex(0, 0, 0);
		problem.mesh_robot->addVertex(0, 0, 0);
		problem.mesh_robot->addVertex(0, 0, 0);
		problem.mesh_robot->addFace(0, 1, 2);
		*/
		problem.mesh_robot->createRAPIDModel();
		problem.mesh_robot->EndObject();

	} else {
		ERROR("unknown gop type "<<gop_type);
		exit(1);
	}

	std::vector<HeapNode<HeapPoint3D> *> border_nodes;
	for (int var = 0; var < problem.border.size(); ++var) {
		MapPoint mp = problem.map_points[problem.border[var]];
		HeapNode<HeapPoint3D> *bhn = new HeapNode<HeapPoint3D>();
		bhn->node_id = problem.border[var];
		bhn->data.x = mp.x;
		bhn->data.y = mp.y;
		bhn->data.z = 0;
		border_nodes.push_back(bhn);
	}
	problem.border_nodes = border_nodes;
	INFO("return problem definition");
	return problem;
}

OP_Prolem<HeapPoint3D> OBSTOPLoader::getSOPDefinitionMapFile(CConfig& config, const std::string& problemFile,
		bool change_end_index_to_last) {
	OP_Prolem<HeapPoint3D> problem;
	std::string gop_type = config.get<std::string>("gop-type");
	gop_type = trim(gop_type);
	INFO("compare gop-type:"<<gop_type);

	if (gop_type.compare("obst_op") == 0) {
		INFO("gop type is sop");
		problem.gop_type = SOP;
		DatasetOBST_OP_MeshObjects<HeapPoint3D> loadedDataset = DatasetLoaderOBSTOP::loadDatasetMapFile(problemFile);
		INFO("has loadedDataset")
		INFO("use budget " << loadedDataset.Tmax);
		INFO("use startIndex " << loadedDataset.startID);
		INFO("use goalIndex " << loadedDataset.goalID);
		problem.name = config.get<std::string>("name");
		problem.startIndex = loadedDataset.startID;
		problem.goalIndex = loadedDataset.goalID;
		problem.name = loadedDataset.name;
		problem.budget = loadedDataset.Tmax;

		std::vector<std::vector<GraphNode<HeapPoint3D>>> clusterNodes;
		for (int var = 0; var < loadedDataset.mesh_objects.size(); ++var) {
			MeshObject* obj = loadedDataset.mesh_objects[var];

			if (obj->getName().compare(ROBOT_NAME) == 0) {
				INFO("has robot object "<<obj->getName());
				obj->setPositionToCenter();
				obj->createRAPIDModel();
				problem.mesh_robot = obj;
			} else if (obj->getName().compare(START_NAME) == 0) {
				INFO("has start object "<<obj->getName());
				obj->setPositionToCenter();
				std::vector<GraphNode<HeapPoint3D>> singleNodeCluster;
				HeapPoint3D data(obj->getPosition().x, obj->getPosition().y, obj->getPosition().z);
				double price = 0;
				int id = clusterNodes.size();
				GraphNode<HeapPoint3D> gn = GraphNode<HeapPoint3D>(data, price, id, id);
				singleNodeCluster.push_back(gn);
				clusterNodes.push_back(singleNodeCluster);
				problem.startIndex = id;
			} else if (obj->getName().compare(GOAL_NAME) == 0) {
				INFO("has goal object "<<obj->getName());
				obj->setPositionToCenter();
				std::vector<GraphNode<HeapPoint3D>> singleNodeCluster;
				HeapPoint3D data(obj->getPosition().x, obj->getPosition().y, obj->getPosition().z);
				double price = 0;
				int id = clusterNodes.size();
				GraphNode<HeapPoint3D> gn = GraphNode<HeapPoint3D>(data, price, id, id);
				singleNodeCluster.push_back(gn);
				clusterNodes.push_back(singleNodeCluster);
				problem.goalIndex = id;
			} else if (obj->getName().compare(0, std::string(TARGET_NAME_START).size(), TARGET_NAME_START) == 0) {
				INFO("has target object "<<obj->getName());
				obj->setPositionToCenter();
				std::vector<GraphNode<HeapPoint3D>> singleNodeCluster;
				HeapPoint3D data(obj->getPosition().x, obj->getPosition().y, obj->getPosition().z);
				double price = loadedDataset.target_reward_map[obj->getName()];
				INFO("loaded price is "<<price);
				int id = clusterNodes.size();
				GraphNode<HeapPoint3D> gn = GraphNode<HeapPoint3D>(data, price, id, id);
				singleNodeCluster.push_back(gn);
				clusterNodes.push_back(singleNodeCluster);
			} else if (obj->getName().compare(BORDER_NAME) == 0) {
				INFO("has border object "<<obj->getName());
				INFO("add border object to obstacles "<<obj->getName());

				obj->createRAPIDModel();
				problem.mesh_obstacles.push_back(obj);
				problem.mesh_border = obj;
			} else {
				INFO("has obstacle "<<obj->getName());
				obj->createRAPIDModel();
				problem.mesh_obstacles.push_back(obj);
			}
			/*
			 std::vector<GraphNode<HeapPoint2D>> singleNodeCluster;
			 GraphNode<HeapPoint2D> gn = loadedDataset.graph[var];
			 gn.cluster_id = var;
			 singleNodeCluster.push_back(gn);
			 clusterNodes.push_back(singleNodeCluster);
			 */
		}

		//exit(1);

		INFO("load dataset problem.startIndex "<<problem.startIndex)
		INFO("load dataset problem.goalIndex "<<problem.goalIndex)

		problem.samples = clusterNodes;
		INFO("load dataset problem.samples.size() "<<problem.samples.size())
		INFO("load dataset problem.mesh_obstacles.size() "<<problem.mesh_obstacles.size())

		for (int var = 0; var < problem.samples.size(); ++var) {
			INFO(
					"target id:"<<problem.samples[var][0].id<<" rew:"<<problem.samples[var][0].reward<<" x,y:"<<problem.samples[var][0].data.x<<" , "<<problem.samples[var][0].data.y<<" , "<<problem.samples[var][0].data.z);
		}

		for (int obstid = 0; obstid < problem.mesh_obstacles.size(); ++obstid) {
			int obstacle_vert_start_id = problem.map_points.size();
			INFO(
					"obstacle "<<problem.mesh_obstacles[obstid]->getName()<<" has "<<problem.mesh_obstacles[obstid]->getFacesNum() <<" faces");
			for (int vertid = 0; vertid < problem.mesh_obstacles[obstid]->getVerticesNum(); ++vertid) {
				Point3D vert = problem.mesh_obstacles[obstid]->getVertices()[vertid];
				MapPoint point;
				point.id = problem.map_points.size();
				point.x = vert.x;
				point.y = vert.y;
				problem.map_points.push_back(point);
			}

			for (int faceid = 0; faceid < problem.mesh_obstacles[obstid]->getFacesNum(); ++faceid) {
				Face face = problem.mesh_obstacles[obstid]->getFaces()[faceid];
				//INFO("face "<<face.id1<<" "<<face.id2<<" "<<face.id3)
				Obstacle obstacle;
				obstacle.point_indexes.push_back(face.id1 + obstacle_vert_start_id);
				obstacle.point_indexes.push_back(face.id2 + obstacle_vert_start_id);
				obstacle.point_indexes.push_back(face.id3 + obstacle_vert_start_id);
				problem.obstacles.push_back(obstacle);
			}
		}

		//for (int mpid = 0; mpid < problem.map_points.size(); ++mpid) {
		//	INFO("map point "<<problem.map_points[mpid].x<<" "<<problem.map_points[mpid].y)
		//}

		//for (int var = 0; var < problem.obstacles.size(); ++var) {
		//	auto obstacle_indexes = problem.obstacles[var].point_indexes;
		//	INFO("obstacle "<<problem.map_points[obstacle_indexes[0]].x<<","<<problem.map_points[obstacle_indexes[0]].y << " - "<<problem.map_points[obstacle_indexes[1]].x<<","<<problem.map_points[obstacle_indexes[1]].y <<" - "<<problem.map_points[obstacle_indexes[2]].x<<","<<problem.map_points[obstacle_indexes[2]].y)
		//}

		std::vector<HeapNode<HeapPoint3D> *> border_node;
		for (int var = 0; var < problem.mesh_border->getVerticesNum(); ++var) {
			HeapNode<HeapPoint3D> *bhn = new HeapNode<HeapPoint3D>();
			bhn->node_id = problem.map_points.size() + var;
			bhn->data.x = problem.mesh_border->getVertices()[var].x;
			bhn->data.y = problem.mesh_border->getVertices()[var].y;
			bhn->data.z = problem.mesh_border->getVertices()[var].z;
			border_node.push_back(bhn);
		}
		problem.border_nodes = border_node;
		//exit(1);

		//problem.mesh_robot;
		//problem.mesh_obstacles;
		//printAllClustersNodes(problem.samples);

	} else {
		ERROR("unknown gop type "<<gop_type);
		exit(1);
	}

	INFO("return problem definition");
	return problem;
}

void OBSTOPLoader::fixClusterNodeRenumbering(std::vector<GraphNode<HeapPoint2D>>& solution, int startIndex,
		int goalIndex, int oldstartIndex, int oldgoalIndex, int deleted_cluster_id, bool numbered_from_one) {

	//for debuging
	INFO("fixClusterNodeRenumbering begin");
	INFO_VAR(startIndex);
	INFO_VAR(goalIndex);
	INFO_VAR(oldstartIndex);
	INFO_VAR(oldgoalIndex);
	INFO_VAR(deleted_cluster_id);
	std::stringstream tourClustersBef;
	for (int var = 0; var < solution.size(); ++var) {
		if (var != 0) {
			tourClustersBef << ",";
		}
		tourClustersBef << solution[var].cluster_id;
	}
	INFO("cluster IDs before: "<<tourClustersBef.str());

	for (int var = 0; var < solution.size(); ++var) {
		if (oldgoalIndex != -1) {
			if (solution[var].cluster_id == goalIndex) {
				solution[var].cluster_id = oldgoalIndex;
				INFO("change "<<goalIndex<< " id to "<<oldgoalIndex<<" at pos "<<var);
				continue;
			}
			if (solution[var].cluster_id == oldgoalIndex && oldstartIndex != oldgoalIndex) {
				solution[var].cluster_id = goalIndex;
				INFO("change "<<oldgoalIndex<< " id to "<<goalIndex<<" at pos "<<var);
				continue;
			}
		}
		if (oldstartIndex != startIndex) {
			if (solution[var].cluster_id == oldstartIndex) {
				solution[var].cluster_id = startIndex;
				INFO("change "<<oldstartIndex<< " id to "<<startIndex<<" at pos "<<var);
				continue;
			}
			if (solution[var].cluster_id == startIndex) {
				solution[var].cluster_id = oldstartIndex;
				INFO("change "<<startIndex<< " id to "<<oldstartIndex<<" at pos "<<var);
				continue;
			}
		}
	}

	if (deleted_cluster_id >= 0) {
		for (int var = 0; var < solution.size(); ++var) {
			if (solution[var].cluster_id >= deleted_cluster_id) {
				solution[var].cluster_id = solution[var].cluster_id + 1;
			}
		}
	}

	//for debuging
	std::stringstream tourClustersAft;
	for (int var = 0; var < solution.size(); ++var) {
		if (var != 0) {
			tourClustersAft << ",";
		}
		tourClustersAft << solution[var].cluster_id;
	}
	INFO("cluster IDs after: "<<tourClustersAft.str());
	INFO("fixClusterNodeRenumbering end");

}

OP_Prolem<HeapPoint2D> OBSTOPLoader::getOP(std::vector<GraphNode<HeapPoint2D>> opNodes) {
	INFO("getOP");

	int S = opNodes.size();
	samples_type_2d samples;
	samples.resize(S);
	for (int nodeID = 0; nodeID < S; ++nodeID) {
		std::vector<GraphNode<HeapPoint2D>> onlyNode(1);
		onlyNode[0] = opNodes[nodeID];
		onlyNode[0].cluster_id = nodeID;
		samples[nodeID] = onlyNode;
	}

	samples_distances_type distances;
	distances.resize(S);
	for (int setid1 = 0; setid1 < S; ++setid1) {
		distances[setid1].resize(S);

		for (int setid2 = 0; setid2 < S; ++setid2) {
			int maxneighID1 = 1;
			distances[setid1][setid2].resize(maxneighID1);
			for (int neighID1 = 0; neighID1 < maxneighID1; ++neighID1) {
				int maxneighID2 = 1;
				distances[setid1][setid2][neighID1].resize(maxneighID2);

				for (int neighID2 = 0; neighID2 < maxneighID2; ++neighID2) {
					distances[setid1][setid2][neighID1][neighID2] = samples[setid1][neighID1].distanceTo(
							samples[setid2][neighID2]);
				}
			}
		}
	}

	OP_Prolem<HeapPoint2D> toReturn;
	toReturn.samples = samples;
	toReturn.distances = distances;
	return toReturn;
}

CoordsVector OBSTOPLoader::getOBSTOPPath(std::vector<IndexSOP> &gopPath, samples_type_2d & nodesAllClusters) {
	//INFO("getOPPath begin");
	int S = gopPath.size();
	//INFO("size "<<S);
	CoordsVector ret(S);
	for (int var = 0; var < S; ++var) {
		//INFO("ci "<<gopPath[var].clusterIndex<<" ni "<<gopPath[var].nodeIndex);
		GraphNode<HeapPoint2D> gn = nodesAllClusters[gopPath[var].clusterIndex][gopPath[var].nodeIndex];
		ret[var] = Coords(gn.data.x, gn.data.y);
	}
	//INFO("getOPPath end");
	return ret;
}

void OBSTOPLoader::printAllClustersNodes(samples_type_2d samples) {
	for (int var = 0; var < samples.size(); ++var) {
		INFO("cluster "<<var<<":")
		for (int var2 = 0; var2 < samples[var].size(); ++var2) {
			INFO(
					"\t node "<<var2<<" id "<<samples[var][var2].id<<" reward "<<samples[var][var2].reward<<" pos "<<samples[var][var2].data.x<<":"<<samples[var][var2].data.y<<" clusted_id "<<samples[var][var2].cluster_id);
		}
	}
}

std::vector<MeshObject*> OBSTOPLoader::create_mesh_obstacles(std::vector<MapPoint> points_vec,
		std::vector<Obstacle> obstacles_vec) {
	INFO("create_mesh_obstacles begin");
	std::vector<MeshObject*> mesh_object_obstacles;
	char * triswitches = (char *) "zQp";
	std::stringstream object_name;
	object_name << "obstacle_object_all";
	MeshObject* mesh_obstacle = new MeshObject(object_name.str());
	mesh_obstacle->BeginObject();
	for (int var = 0; var < points_vec.size(); ++var) {
		mesh_obstacle->addVertex(points_vec[var].x, points_vec[var].y, 0);
	}

	for (int var = 0; var < obstacles_vec.size(); ++var) {
		INFO("create object " << var)

		Obstacle testing_obstacle = obstacles_vec[var];
		int num_outvertices = testing_obstacle.point_indexes.size();

		struct triangulateio in;
		struct triangulateio out;
		struct triangulateio vorout;

		//fill in struct
		in.numberofpoints = num_outvertices;
		in.pointlist = (REAL*) trimalloc((int) (num_outvertices * 2 * sizeof(REAL)));
		in.pointmarkerlist = (int*) trimalloc((int) (num_outvertices * sizeof(int)));
		in.numberofsegments = num_outvertices;
		in.segmentlist = (int*) trimalloc((int) (num_outvertices * 2 * sizeof(int)));
		in.numberofpointattributes = 0;
		in.segmentmarkerlist = NULL;
		in.numberofholes = 0;
		in.numberofregions = 0;

		in.pointattributelist = NULL;
		for (int var = 0; var < num_outvertices; ++var) {
			int point_index = testing_obstacle.point_indexes[var];
			in.pointlist[2 * var] = points_vec[point_index].x;
			in.pointlist[2 * var + 1] = points_vec[point_index].y;
			//mesh_obstacle->addVertex(points_vec[point_index].x, points_vec[point_index].y, 0);
			//INFO("add vertex "<<var<<" point index "<<point_index);
			in.pointmarkerlist[var] = point_index;
			in.segmentlist[var * 2] = var;
			in.segmentlist[var * 2 + 1] = var + 1;
		}
		in.segmentlist[(num_outvertices - 1) * 2 + 1] = 0;

		//INFO("malloc done")
		//fill out strucdt
		out.pointlist = NULL;
		out.pointmarkerlist = NULL;
		out.trianglelist = NULL;
		out.neighborlist = NULL;
		out.segmentlist = NULL;
		out.segmentmarkerlist = NULL;
		//create corout struct

		//INFO("before triangulation")
		triangulate(triswitches, &in, &out, &vorout);
		//INFO("after triangulation")

		for (int var = 0; var < out.numberoftriangles; ++var) {
			int cor1 = out.trianglelist[var * 3];
			int cor2 = out.trianglelist[var * 3 + 1];
			int cor3 = out.trianglelist[var * 3 + 2];
			//INFO("real "<<cor1<<" "<<cor2<<" "<<cor3)
			//INFO("orig "<<out.pointmarkerlist[cor1]<<" "<<out.pointmarkerlist[cor2]<<" "<<out.pointmarkerlist[cor3])
			//mesh_obstacle->addFace(cor1, cor2, cor3);
			mesh_obstacle->addFace(out.pointmarkerlist[cor1], out.pointmarkerlist[cor2], out.pointmarkerlist[cor3]);
			//INFO("add face "<<var<<" with vertices "<<cor1<<" "<<cor2<<" "<<cor3)
			Obstacle tri_obstacle;
			tri_obstacle.point_indexes.push_back(out.pointmarkerlist[cor1]);
			tri_obstacle.point_indexes.push_back(out.pointmarkerlist[cor2]);
			tri_obstacle.point_indexes.push_back(out.pointmarkerlist[cor3]);
			//mesh_obstacle.ad

		}

		//INFO("free memory")
		//ERROR("free memory error")

		//free memory
		trifree(in.pointlist);
		trifree(in.pointmarkerlist);
		trifree(in.segmentlist);
		trifree(out.pointlist);
		trifree(out.pointmarkerlist);
		trifree(out.trianglelist);
		trifree(out.neighborlist);
		trifree(out.segmentlist);
		trifree(out.segmentmarkerlist);
		//INFO("free memory done")
	}

	mesh_obstacle->EndObject();
	mesh_obstacle->createRAPIDModel();

	INFO("rapid model created");
	mesh_object_obstacles.push_back(mesh_obstacle);
	INFO("create_mesh_obstacles end");
	return mesh_object_obstacles;
}
