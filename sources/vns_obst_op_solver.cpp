/*
 * VNSVNSGOPPath.cpp
 *
 *  Created on: 22. 3. 2016
 *      Author: Robert Penicka
 */

#include "vns_obst_op_solver.h"

using crl::logger;

using namespace op;
using namespace crl;
using namespace crl::gui;
using namespace opendubins;

namespace op {

template<>
void VnsPrmOPSolver<HeapPoint2D>::save_prm_points(std::string filename) {
	INFO("save_prm_point_path 2d");

	std::string dir = output;
	std::string file = getOutputIterPath(filename, dir);
	assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
	std::ofstream out(file.c_str());
	assert_io(out.good(), "Cannot create path '" + file + "'");
	std::string delimiter = ", ";

	if (out.is_open()) {
		std::vector<HeapNode<HeapPoint2D>*> points = prm->get_points();
		for (int var = 0; var < points.size(); ++var) {
			out << points[var]->data.x << delimiter << points[var]->data.y << std::endl;
		}
		out.close();
	} else {
		std::cerr << "Cannot open " << filename << std::endl;
	}
}

template<>
void VnsPrmOPSolver<HeapPoint2DHeading>::save_prm_points(std::string filename) {
	INFO("save_prm_point_path 2d heading");

	std::string dir = output;
	std::string file = getOutputIterPath(filename, dir);
	assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
	std::ofstream out(file.c_str());
	assert_io(out.good(), "Cannot create path '" + file + "'");
	std::string delimiter = ", ";

	if (out.is_open()) {
		std::vector<HeapNode<HeapPoint2DHeading>*> points = prm->get_points();
		for (int var = 0; var < points.size(); ++var) {
			out << points[var]->data.x << delimiter << points[var]->data.y << delimiter << points[var]->data.phi
					<< std::endl;
		}
		out.close();
	} else {
		std::cerr << "Cannot open " << filename << std::endl;
	}
}

template<>
void VnsPrmOPSolver<HeapPoint3D>::save_prm_points(std::string filename) {
	INFO("save_prm_point_path 2d");

	std::string dir = output;
	std::string file = getOutputIterPath(filename, dir);
	assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
	std::ofstream out(file.c_str());
	assert_io(out.good(), "Cannot create path '" + file + "'");
	std::string delimiter = ", ";

	if (out.is_open()) {
		std::vector<HeapNode<HeapPoint3D>*> points = prm->get_points();
		for (int var = 0; var < points.size(); ++var) {
			out << points[var]->data.x << delimiter << points[var]->data.y << delimiter << points[var]->data.z
					<< std::endl;
		}
		out.close();
	} else {
		std::cerr << "Cannot open " << filename << std::endl;
	}
}

template<>
void VnsPrmOPSolver<HeapPoint2D>::save_prm_point_path(std::string filename, VnsSopPath<HeapPoint2D> * toShow) {
	//INFO_VAR(returnedPath.size());
	VnsSopPath<HeapPoint2D> path_to_save = this->tourVNSGOPPath;
	if (toShow != NULL) {
		path_to_save = *toShow;
	}
	std::vector<IndexSOP> returnedPath = path_to_save.getPath();

	if (returnedPath.size() >= 2) {
		std::string dir = output;
		std::string file = getOutputIterPath(filename, dir);
		assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream out(file.c_str());
		assert_io(out.good(), "Cannot create path '" + file + "'");
		std::string delimiter = ", ";
		if (out.is_open()) {
			std::vector<HeapNode<HeapPoint2D>> plan_all;
			for (int var = 1; var < returnedPath.size(); ++var) {
				int node_id_from =
						nodesAllClusters[returnedPath[var - 1].clusterIndex][returnedPath[var - 1].nodeIndex].id;
				int node_id_to = nodesAllClusters[returnedPath[var].clusterIndex][returnedPath[var].nodeIndex].id;
				plan_with_length<HeapPoint2D> found_path = prm->plan(node_id_from, node_id_to);
				if (var == 1) {
					plan_all.insert(std::end(plan_all), std::begin(found_path.plan), std::end(found_path.plan));
				} else {
					plan_all.insert(std::end(plan_all), std::begin(found_path.plan) + 1, std::end(found_path.plan));
				}
			}
			for (int var = 0; var < plan_all.size(); ++var) {
				out << plan_all[var].data.x << delimiter << plan_all[var].data.y << std::endl;
			}
			out.close();
		} else {
			std::cerr << "Cannot open " << filename << std::endl;
		}

	}
}

template<>
void VnsPrmOPSolver<HeapPoint2DHeading>::save_prm_point_path(std::string filename,
		VnsSopPath<HeapPoint2DHeading> * toShow) {
	VnsSopPath<HeapPoint2DHeading> path_to_save = this->tourVNSGOPPath;
	if (toShow != NULL) {
		path_to_save = *toShow;
	}
	std::vector<IndexSOP> returnedPath = path_to_save.getPath();
	//INFO_VAR(returnedPath.size());
	if (returnedPath.size() >= 2) {
		std::string dir = output;
		std::string file = getOutputIterPath(filename, dir);
		assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream out(file.c_str());
		assert_io(out.good(), "Cannot create path '" + file + "'");
		std::string delimiter = ", ";
		if (out.is_open()) {
			std::vector<HeapNode<HeapPoint2DHeading>> plan_all;
			for (int var = 1; var < returnedPath.size(); ++var) {
				int node_id_from =
						nodesAllClusters[returnedPath[var - 1].clusterIndex][returnedPath[var - 1].nodeIndex].id;
				int node_id_to = nodesAllClusters[returnedPath[var].clusterIndex][returnedPath[var].nodeIndex].id;
				plan_with_length<HeapPoint2DHeading> found_path = prm->plan(node_id_from, node_id_to);
				if (var == 1) {
					plan_all.insert(std::end(plan_all), std::begin(found_path.plan), std::end(found_path.plan));
				} else {
					plan_all.insert(std::end(plan_all), std::begin(found_path.plan) + 1, std::end(found_path.plan));
				}
			}
			for (int var = 0; var < plan_all.size(); ++var) {
				out << plan_all[var].data.x << delimiter << plan_all[var].data.y << delimiter << plan_all[var].data.phi
						<< std::endl;
			}
			out.close();
		} else {
			std::cerr << "Cannot open " << filename << std::endl;
		}

	}
}

template<>
void VnsPrmOPSolver<HeapPoint3D>::save_prm_point_path(std::string filename, VnsSopPath<HeapPoint3D> * toShow) {
	//INFO_VAR(returnedPath.size());
	VnsSopPath<HeapPoint3D> path_to_save = this->tourVNSGOPPath;
	if (toShow != NULL) {
		path_to_save = *toShow;
	}
	std::vector<IndexSOP> returnedPath = path_to_save.getPath();

	if (returnedPath.size() >= 2) {
		std::string dir = output;
		std::string file = getOutputIterPath(filename, dir);
		assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream out(file.c_str());
		assert_io(out.good(), "Cannot create path '" + file + "'");
		std::string delimiter = ", ";
		if (out.is_open()) {
			std::vector<HeapNode<HeapPoint3D>> plan_all;
			for (int var = 1; var < returnedPath.size(); ++var) {
				int node_id_from =
						nodesAllClusters[returnedPath[var - 1].clusterIndex][returnedPath[var - 1].nodeIndex].id;
				int node_id_to = nodesAllClusters[returnedPath[var].clusterIndex][returnedPath[var].nodeIndex].id;
				plan_with_length<HeapPoint3D> found_path = prm->plan(node_id_from, node_id_to);
				if (var == 1) {
					plan_all.insert(std::end(plan_all), std::begin(found_path.plan), std::end(found_path.plan));
				} else {
					plan_all.insert(std::end(plan_all), std::begin(found_path.plan) + 1, std::end(found_path.plan));
				}
			}
			for (int var = 0; var < plan_all.size(); ++var) {
				out << plan_all[var].data.x << delimiter << plan_all[var].data.y << delimiter << plan_all[var].data.z << std::endl;
			}
			out.close();
		} else {
			std::cerr << "Cannot open " << filename << std::endl;
		}

	}
}

template<>
void VnsPrmOPSolver<HeapPoint2D>::fillCityNodes(OP_Prolem<HeapPoint3D> &problem) {
	INFO("fillCityNodes begin");
	nodesAllClusters.resize(problem.samples.size());
	//this->nodesAllClusters = problem.samples;
	for (int clid = 0; clid < nodesAllClusters.size(); ++clid) {
		nodesAllClusters[clid].resize(problem.samples[clid].size());

		for (int nodeid = 0; nodeid < nodesAllClusters[clid].size(); ++nodeid) {
			nodesAllClusters[clid][nodeid].cluster_id = problem.samples[clid][0].cluster_id;
			nodesAllClusters[clid][nodeid].id = problem.samples[clid][0].id;
			nodesAllClusters[clid][nodeid].reward = problem.samples[clid][0].reward;
			nodesAllClusters[clid][nodeid].data.x = problem.samples[clid][0].data.x;
			nodesAllClusters[clid][nodeid].data.y = problem.samples[clid][0].data.y;
		}
		maximalRewardAll += nodesAllClusters[clid][0].reward;
		cluster_rewards[clid] = nodesAllClusters[clid][0].reward;
	}
	INFO("nodesAllClusters filled")

	cities_nodes.reserve(nodesAllClusters.size());
	for (int var = 0; var < nodesAllClusters.size(); ++var) {
		HeapNode<HeapPoint2D> *city_node = new HeapNode<HeapPoint2D>();
		city_node->data.x = nodesAllClusters[var][0].data.x;
		city_node->data.y = nodesAllClusters[var][0].data.y;
		city_node->node_id = nodesAllClusters[var][0].id;
		city_node->cluster_id = nodesAllClusters[var][0].cluster_id;
		city_node->city_node = true;
		cities_nodes.push_back(city_node);
	}
	INFO("city nodes set");
	INFO("fillCityNodes end");
}

template<>
void VnsPrmOPSolver<HeapPoint2DHeading>::fillCityNodes(OP_Prolem<HeapPoint3D> &problem) {
	INFO("fillCityNodes begin");
	INFO_VAR(dubins_resolution);
	int act_node_id = 0;
	nodesAllClusters.resize(num_clusters);
	for (int clid = 0; clid < nodesAllClusters.size(); ++clid) {
		//nodesAllClusters
		maximalRewardAll += problem.samples[clid][0].reward;
		cluster_rewards[clid] = problem.samples[clid][0].reward;
		nodesAllClusters[clid].resize(dubins_resolution);
		for (int nodeid = 0; nodeid < dubins_resolution; ++nodeid) {
			nodesAllClusters[clid][nodeid].cluster_id = problem.samples[clid][0].cluster_id;
			nodesAllClusters[clid][nodeid].id = act_node_id;
			act_node_id += 1;
			nodesAllClusters[clid][nodeid].reward = problem.samples[clid][0].reward;
			nodesAllClusters[clid][nodeid].data.x = problem.samples[clid][0].data.x;
			nodesAllClusters[clid][nodeid].data.y = problem.samples[clid][0].data.y;
			nodesAllClusters[clid][nodeid].data.phi = nodeid * M_2PI / dubins_resolution;
		}
	}
	INFO("nodesAllClusters filled");

	num_clusters = nodesAllClusters.size();

	cities_nodes.reserve(nodesAllClusters.size() * dubins_resolution);
	for (int var = 0; var < nodesAllClusters.size(); ++var) {
		for (int head_id = 0; head_id < dubins_resolution; ++head_id) {
			HeapNode<HeapPoint2DHeading> *city_node = new HeapNode<HeapPoint2DHeading>();
			city_node->data.x = nodesAllClusters[var][head_id].data.x;
			city_node->data.y = nodesAllClusters[var][head_id].data.y;
			city_node->data.phi = nodesAllClusters[var][head_id].data.phi;
			city_node->data.radius = dubins_radius;
			city_node->node_id = nodesAllClusters[var][head_id].id;
			city_node->cluster_id = nodesAllClusters[var][head_id].cluster_id;
			city_node->city_node = true;
			cities_nodes.push_back(city_node);
		}
	}
	INFO("city nodes set");
	this->prm->set_num_headings(dubins_resolution);
	INFO("fillCityNodes end");
}

template<>
void VnsPrmOPSolver<HeapPoint3D>::fillCityNodes(OP_Prolem<HeapPoint3D> &problem) {
	INFO("fillCityNodes begin");
	this->nodesAllClusters = problem.samples;
	for (int clid = 0; clid < nodesAllClusters.size(); ++clid) {
		maximalRewardAll += nodesAllClusters[clid][0].reward;
		cluster_rewards[clid] = nodesAllClusters[clid][0].reward;
	}
	INFO("nodesAllClusters filled")

	cities_nodes.reserve(nodesAllClusters.size());
	for (int var = 0; var < nodesAllClusters.size(); ++var) {
		HeapNode<HeapPoint3D> *city_node = new HeapNode<HeapPoint3D>();
		city_node->data.x = nodesAllClusters[var][0].data.x;
		city_node->data.y = nodesAllClusters[var][0].data.y;
		city_node->data.z = nodesAllClusters[var][0].data.z;
		city_node->node_id = nodesAllClusters[var][0].id;
		city_node->cluster_id = nodesAllClusters[var][0].cluster_id;
		city_node->city_node = true;
		cities_nodes.push_back(city_node);
	}
	INFO("city nodes set");
	INFO("fillCityNodes end");
}

template<>
void VnsPrmOPSolver<HeapPoint2D>::drawVisibilityPath(int usleepTime, std::vector<HeapNode<HeapPoint2D>> * toShow) {
	if (canvas) {
		//INFO("clear path");
		*canvas << canvas::CLEAR << "path" << "path";
		std::vector<HeapNode<HeapPoint2D>> returnedPath = *toShow;
		CShape greenLine("green", "green", 3, 3);
		for (int var = 1; var < returnedPath.size(); ++var) {
			*canvas << canvas::LINE << greenLine;
			*canvas << returnedPath[var - 1].data.x << returnedPath[var - 1].data.y;
			*canvas << returnedPath[var].data.x << returnedPath[var].data.y;

		}
		*canvas << canvas::END;
		canvas->redraw();
	}
}

template<>
void VnsPrmOPSolver<HeapPoint2DHeading>::drawVisibilityPath(int usleepTime,
		std::vector<HeapNode<HeapPoint2DHeading>> * toShow) {
	if (canvas) {
		//INFO("drawVisibilityPath dubins begin");
		*canvas << canvas::CLEAR << "path" << "path";
		std::vector<HeapNode<HeapPoint2DHeading>> returnedPath = *toShow;

		CShape greenLine("green", "green", 3, 3);
		State previous_state;
		for (int var = 1; var < returnedPath.size(); ++var) {

			State start_state(returnedPath[var - 1].data.x, returnedPath[var - 1].data.y,
					returnedPath[var - 1].data.phi);
			State stop_state(returnedPath[var].data.x, returnedPath[var].data.y, returnedPath[var].data.phi);
			//INFO_VAR(start_state)
			//INFO_VAR(stop_state)
			if (var > 1) {
				if (previous_state.point.x != start_state.point.x || previous_state.point.y != start_state.point.y
						|| previous_state.ang != start_state.ang) {
					ERROR("not continuous points");
					exit(1);
				}
			}
			previous_state = stop_state;
			//INFO("dubins from "<<start_state<<" radius "<<returnedPath[var - 1].data.radius<<" to "<<stop_state<<" radius "<<returnedPath[var].data.radius)
			double radius = returnedPath[var - 1].data.radius;
			Dubins dub(start_state, stop_state, radius);
			*canvas << greenLine << dub;

		}
		*canvas << canvas::END;
		canvas->redraw();
		//INFO("drawVisibilityPath dubins end");
	}
}

template<>
void VnsPrmOPSolver<HeapPoint3D>::drawVisibilityPath(int usleepTime, std::vector<HeapNode<HeapPoint3D>> * toShow) {
	if (canvas) {
		//INFO("clear path");
		*canvas << canvas::CLEAR << "path" << "path";
		std::vector<HeapNode<HeapPoint3D>> returnedPath = *toShow;
		CShape greenLine("green", "green", 3, 3);
		for (int var = 1; var < returnedPath.size(); ++var) {
			*canvas << canvas::LINE << greenLine;
			*canvas << returnedPath[var - 1].data.x << returnedPath[var - 1].data.y;
			*canvas << returnedPath[var].data.x << returnedPath[var].data.y;

		}
		*canvas << canvas::END;
		canvas->redraw();
	}
}

template<>
std::vector<std::vector<double>> VnsPrmOPSolver<HeapPoint2D>::getTrajectoryPointVectors(
		std::vector<HeapPoint2D> samples_traj) {
	std::vector<std::vector<double>> vec;
	for (int var1 = 0; var1 < samples_traj.size(); ++var1) {
		std::vector<double> single_vec = { samples_traj[var1].x, samples_traj[var1].y, this->fly_altitude, 0 };
		vec.push_back(single_vec);
	}
	return vec;
}

template<>
std::vector<std::vector<double>> VnsPrmOPSolver<HeapPoint2DHeading>::getTrajectoryPointVectors(
		std::vector<HeapPoint2DHeading> samples_traj) {
	std::vector<std::vector<double>> vec;
	for (int var1 = 0; var1 < samples_traj.size(); ++var1) {
		std::vector<double> single_vec = { samples_traj[var1].x, samples_traj[var1].y, this->fly_altitude,
				samples_traj[var1].phi };
		vec.push_back(single_vec);
	}
	return vec;
}

template<>
std::vector<std::vector<double>> VnsPrmOPSolver<HeapPoint3D>::getTrajectoryPointVectors(
		std::vector<HeapPoint3D> samples_traj) {
	std::vector<std::vector<double>> vec;
	for (int var1 = 0; var1 < samples_traj.size(); ++var1) {
		std::vector<double> single_vec = { samples_traj[var1].x, samples_traj[var1].y, this->fly_altitude, 0 };
		vec.push_back(single_vec);
	}
	return vec;
}

}
