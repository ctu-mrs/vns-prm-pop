/*
 * DatasetLoader.cpp
 *
 *  Created on: 15. 3. 2016
 *      Author: Robert Penicka
 */

#include "dataset_loader_obst.h"

#define HEADER_INFO "[INFO]"
#define HEADER_MAP_POINTS "[MAP_POINTS]"
#define HEADER_MAP_BORDER "[MAP_BORDER]"
#define HEADER_MAP_OBSTACLE "[MAP_OBSTACLE]"
#define HEADER_CITY_POINTS "[CITY_POINTS]"
#define HEADER_MAP_CONVEX_REGION "[MAP_CONVEX_REGION]"
#define HEADER_MAP_VISIBILITY_GRAPH "[MAP_VISIBILITY_GRAPH]"
#define HEADER_CITY_VISIBILITY "[CITY_VISIBILITY]"
#define HEADER_TARGET_REWARDS "[TARGET_REWARDS]"

#define MAP_FILE_FORMAT "MAP_FILE"
#define MAP_POINTS_CITY_POINTS_FORMAT "MAP_POINTS_CITY_POINTS"

#define INFO_MAP_FILE "MAP_FILE"
#define INFO_FORMAT "FORMAT"
#define INFO_COMMENT "COMMENT"
#define INFO_NAME "NAME"
#define INFO_TMAX "TMAX"
#define INFO_TYPE "TYPE"
#define INFO_VERSION "VERSION"
#define START_INDEX "START_INDEX"
#define END_INDEX "END_INDEX"

enum LoadingPart {
	none_loading,
	info,
	map_points,
	map_border,
	map_obstacle,
	city_points,
	convex_region,
	map_visibility_graph,
	city_visibility_graph,
	target_rewards
};

using crl::logger;

DatasetLoaderOBSTOP::DatasetLoaderOBSTOP() {

}

DatasetLoaderOBSTOP::~DatasetLoaderOBSTOP() {

}

DatasetOBST_OP<HeapPoint3D> DatasetLoaderOBSTOP::loadDatasetMapCityPoints(std::string filename) {
	INFO("DatasetLoaderOBSTOP::loadDataset");
	DatasetOBST_OP<HeapPoint3D> loadedDataset;
	int depot_id = 0;
	loadedDataset.startID = depot_id;
	loadedDataset.goalID = depot_id;
	std::vector<MapPoint> map_nodes;
	std::vector<GraphNode<HeapPoint3D>> target_nodes;
	std::vector<int> map_border_indexes;
	LoadingPart actual_loading = none_loading;
	MapFormat file_format = none_format;
	std::vector<Obstacle> obstacles;
	Obstacle actual_obstacle;
	std::vector<Obstacle> convex_regions;
	Obstacle actual_convex_region;
	std::vector<Visibility> city2map_visibility_lists;
	std::vector<Visibility> city2city_visibility_lists;
	std::vector<Visibility> map_visibility_lists;
	Visibility actual_visibility_list;
	VisibilityVertexType visibility_vertex_type;

	std::ifstream in(filename.c_str(), std::ifstream::in);

	if (!in) {
		std::cerr << "Cannot open " << filename << std::endl;
	} else {
		std::string line;
		unsigned int lineNumber = 0;
		unsigned int actualGNID = 0;
		bool dataSection, setSection = false;
		int dimension;
		int loaded_map_points = 0;
		int visibility_loading_id = -1;

		while (getline(in, line)) {
			lineNumber++;
			line = trim(line);
			int delimiterPos = line.find("=");
			//INFO("line:"<<line);
			if (delimiterPos != std::string::npos) {
				std::string bef = line.substr(0, delimiterPos);
				std::string aftr("");
				//INFO_VAR(line.size());
				//INFO(delimiterPos + 1);
				if (line.size() > delimiterPos + 1) {
					//INFO("subs");
					aftr = line.substr(delimiterPos + 1);
				}
				aftr = trim(aftr);
				bef = trim(bef);

				if (!std::strcmp(bef.c_str(), START_INDEX)) {
					loadedDataset.startID = std::stoi(aftr);
					INFO("loading START_INDEX "<<loadedDataset.startID);
				} else if (!std::strcmp(bef.c_str(), END_INDEX)) {
					loadedDataset.goalID = std::stoi(aftr);
					INFO("loading END_INDEX "<<loadedDataset.goalID);
				}
			}

			if (!std::strcmp(line.c_str(), HEADER_INFO)) {
				actual_loading = info;
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_MAP_POINTS)) {
				actual_loading = map_points;
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_MAP_BORDER)) {
				actual_loading = map_border;
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_MAP_OBSTACLE)) {
				actual_loading = map_obstacle;
				actual_obstacle = Obstacle();
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_CITY_POINTS)) {
				actual_loading = city_points;
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_MAP_CONVEX_REGION)) {
				actual_loading = convex_region;
				actual_convex_region = Obstacle();
				//INFO("new convex region:")
				continue;
			} else if (!std::strcmp(line.c_str(),
			HEADER_MAP_VISIBILITY_GRAPH)) {
				actual_loading = map_visibility_graph;
				actual_visibility_list = Visibility();
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_CITY_VISIBILITY)) {
				actual_loading = city_visibility_graph;
				actual_visibility_list = Visibility();
				continue;
			} else if (line.empty()) {
				if (actual_loading == map_obstacle) {
					//INFO("end obstacle region.")
					obstacles.push_back(actual_obstacle);
				}
				if (actual_loading == convex_region) {
					//INFO("end convex region.")
					convex_regions.push_back(actual_convex_region);
				}
				if (actual_loading == map_visibility_graph) {
					//INFO("end visibility list.")
					if (visibility_vertex_type == city2map_visibility_vertex) {
						actual_visibility_list.visibility_type = city2map_visibility_vertex;
						city2map_visibility_lists.push_back(actual_visibility_list);
					} else {
						actual_visibility_list.visibility_type = map_visibility_vertex;
						map_visibility_lists.push_back(actual_visibility_list);
					}
				}
				if (actual_loading == city_visibility_graph) {
					actual_visibility_list.visibility_type = city2city_visibility_vertex;
					city2city_visibility_lists.push_back(actual_visibility_list);
				}
				actual_loading = none_loading;
				continue;
			}

			if (actual_loading == info) {
				//INFO("loading info");
				int delimiterPos = line.find("=");
				std::string bef("");
				std::string aftr("");
				if (delimiterPos != std::string::npos) {
					bef = line.substr(0, delimiterPos);
					if (line.size() > delimiterPos + 1) {
						aftr = line.substr(delimiterPos + 1);
					}
					aftr = trim(aftr);
					bef = trim(bef);
					//INFO(" aftr "<<aftr<<" bef "<<bef)
				}

				if (!std::strcmp(bef.c_str(), INFO_NAME)) {
					loadedDataset.name = aftr;
					INFO("loaded name "<<loadedDataset.name);
				}
				if (!std::strcmp(bef.c_str(), INFO_TMAX)) {
					std::istringstream s(aftr);
					double tmax_d;
					s >> tmax_d;
					loadedDataset.Tmax = tmax_d;
					INFO("loaded tmax "<<loadedDataset.Tmax);
				}
				if (!std::strcmp(bef.c_str(), INFO_FORMAT)) {
					if (!std::strcmp(aftr.c_str(), MAP_FILE_FORMAT)) {
						INFO("using "<<MAP_FILE_FORMAT);
						file_format = map_file;
					} else if (!std::strcmp(aftr.c_str(),
					MAP_POINTS_CITY_POINTS_FORMAT)) {
						INFO("using "<<MAP_POINTS_CITY_POINTS_FORMAT);
						file_format = map_points_city_points;
					} else {
						ERROR(MAP_FILE_FORMAT<<" not specified or bad value "<<aftr.c_str());
						exit(1);
					}
				}

			}

			if (actual_loading == map_points) {
				std::istringstream s(line);
				MapPoint newMP;
				s >> newMP.id;
				s >> newMP.x;
				s >> newMP.y;
				map_nodes.push_back(newMP);
				//INFO("add map point "<<newMP.id<<" "<<newMP.x<<" "<<newMP.y)
				loaded_map_points++;
				if (map_nodes.size() != loaded_map_points || newMP.id + 1 != loaded_map_points) {
					ERROR("wrong id assignment to dataset nodes");
					ERROR(newMP.id<<" "<<newMP.x<<" "<<newMP.y)
					exit(1);
				}
			}

			if (actual_loading == map_border) {
				std::istringstream s(line);
				int border_idx;
				s >> border_idx;
				map_border_indexes.push_back(border_idx);
			}

			if (actual_loading == map_obstacle) {
				std::istringstream s(line);
				int obstacle_idx;
				s >> obstacle_idx;
				actual_obstacle.point_indexes.push_back(obstacle_idx);
			}

			if (actual_loading == city_points) {
				std::istringstream s(line);
				GraphNode<HeapPoint3D> newGN;
				s >> newGN.id;
				s >> newGN.data.x;
				s >> newGN.data.y;
				s >> newGN.reward;
				newGN.data.z = 0;
				//INFO("add city point "<<newGN.id<<" "<<newGN.data.x<<" "<<newGN.data.y<<" "<<newGN.reward)
				target_nodes.push_back(newGN);
			}

			if (actual_loading == convex_region) {
				std::istringstream s(line);
				if (line.find("REGION_LABEL") != std::string::npos) {
					std::string str;
					int label_num;
					s >> str;
					s >> label_num;
					//INFO("REGION_LABEL "<<label_num)
				} else {
					int region_idx;
					s >> region_idx;
					//INFO(region_idx)
					actual_convex_region.point_indexes.push_back(region_idx);
				}
			}

			if (actual_loading == map_visibility_graph) {
				std::istringstream s(line);
				if (line.find("MAP_VERTEX") != std::string::npos) {
					std::string str;
					s >> str;
					s >> visibility_loading_id;
					visibility_vertex_type = map_visibility_vertex;
					//INFO("MAP_VERTEX "<<visibility_loading_id)
					if (map_visibility_lists.size() != visibility_loading_id) {
						ERROR("bad MAP_VERTEX id "<<visibility_loading_id);
						exit(1);
					}
				} else if (line.find("CITY_VERTEX") != std::string::npos) {
					std::string str;
					s >> str;
					s >> visibility_loading_id;
					visibility_vertex_type = city2map_visibility_vertex;
					//INFO("CITY_VERTEX "<<visibility_loading_id)
					if (city2map_visibility_lists.size() != visibility_loading_id) {
						ERROR("bad CITY_VERTEX id "<<visibility_loading_id);
						exit(1);
					}
				} else {
					int point_idx;
					s >> point_idx;
					//INFO(point_idx)
					actual_visibility_list.point_indexes.push_back(point_idx);
				}
			}

			if (actual_loading == city_visibility_graph) {
				std::istringstream s(line);
				if (line.find("CITY_VERTEX") != std::string::npos) {
					std::string str;
					s >> str;
					s >> visibility_loading_id;
					visibility_vertex_type = city2city_visibility_vertex;
					//INFO("CITY_VERTEX "<<visibility_loading_id)
					if (city2city_visibility_lists.size() != visibility_loading_id) {
						ERROR("bad CITY_VERTEX id "<<visibility_loading_id);
						exit(1);
					}
				} else {
					int point_idx;
					s >> point_idx;
					//INFO(point_idx)
					actual_visibility_list.point_indexes.push_back(point_idx);
				}
			}

		}
		INFO("loaded "<<loaded_map_points<<" map points");
		INFO("with "<<obstacles.size()<<" obstacles");
		INFO("and "<<target_nodes.size()<<" target locations");
	}

	loadedDataset.obstacles = obstacles;
	loadedDataset.map_points = map_nodes;
	loadedDataset.border = map_border_indexes;
	loadedDataset.graph = target_nodes;
	loadedDataset.convex_regions = convex_regions;
	loadedDataset.city2map_visibility_lists = city2map_visibility_lists;
	loadedDataset.map_visibility_lists = map_visibility_lists;
	loadedDataset.city2city_visibility_lists = city2city_visibility_lists;
	//exit(1);
	return loadedDataset;
}

DatasetOBST_OP_MeshObjects<HeapPoint3D> DatasetLoaderOBSTOP::loadDatasetMapFile(std::string filename) {
	INFO("DatasetLoaderOBSTOP::loadDatasetMapFile");
	DatasetOBST_OP_MeshObjects<HeapPoint3D> loadedDataset;
	int depot_id = 0;
	loadedDataset.startID = depot_id;
	loadedDataset.goalID = depot_id;
	std::vector<MapPoint> map_nodes;
	std::vector<GraphNode<HeapPoint2D>> target_nodes;
	std::vector<int> map_border_indexes;
	LoadingPart actual_loading = none_loading;
	MapFormat file_format = none_format;
	std::vector<MeshObject*> mesh_objects;
	Visibility actual_visibility_list;
	VisibilityVertexType visibility_vertex_type;
	std::map<std::string, double> target_reward_map;

	std::ifstream in(filename.c_str(), std::ifstream::in);

	if (!in) {
		std::cerr << "Cannot open " << filename << std::endl;
	} else {
		std::string line;
		unsigned int lineNumber = 0;
		unsigned int actualGNID = 0;
		bool dataSection, setSection = false;
		int dimension;
		int loaded_map_points = 0;
		int visibility_loading_id = -1;

		while (getline(in, line)) {
			lineNumber++;
			line = trim(line);
			int delimiterPos = line.find("=");
			//INFO("line:"<<line);
			if (delimiterPos != std::string::npos) {
				std::string bef = line.substr(0, delimiterPos);
				std::string aftr("");
				//INFO_VAR(line.size());
				//INFO(delimiterPos + 1);
				if (line.size() > delimiterPos + 1) {
					//INFO("subs");
					aftr = line.substr(delimiterPos + 1);
				}
				aftr = trim(aftr);
				bef = trim(bef);

				if (!std::strcmp(bef.c_str(), START_INDEX)) {
					loadedDataset.startID = std::stoi(aftr);
					INFO("loading START_INDEX "<<loadedDataset.startID);
				} else if (!std::strcmp(bef.c_str(), END_INDEX)) {
					loadedDataset.goalID = std::stoi(aftr);
					INFO("loading END_INDEX "<<loadedDataset.goalID);
				}
			}

			if (!std::strcmp(line.c_str(), HEADER_INFO)) {
				actual_loading = info;
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_MAP_POINTS)) {
				actual_loading = map_points;
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_MAP_BORDER)) {
				actual_loading = map_border;
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_CITY_POINTS)) {
				actual_loading = city_points;
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_TARGET_REWARDS)) {
				actual_loading = target_rewards;
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_MAP_VISIBILITY_GRAPH)) {
				actual_loading = map_visibility_graph;
				actual_visibility_list = Visibility();
				continue;
			} else if (!std::strcmp(line.c_str(), HEADER_CITY_VISIBILITY)) {
				actual_loading = city_visibility_graph;
				actual_visibility_list = Visibility();
				continue;
			} else if (line.empty()) {
				actual_loading = none_loading;
				continue;
			}

			if (actual_loading == info) {
				//INFO("loading info");
				int delimiterPos = line.find("=");
				std::string bef("");
				std::string aftr("");
				if (delimiterPos != std::string::npos) {
					bef = line.substr(0, delimiterPos);
					if (line.size() > delimiterPos + 1) {
						aftr = line.substr(delimiterPos + 1);
					}
					aftr = trim(aftr);
					bef = trim(bef);
					//INFO(" aftr "<<aftr<<" bef "<<bef)
				}

				if (!std::strcmp(bef.c_str(), INFO_NAME)) {
					loadedDataset.name = aftr;
					INFO("loaded name "<<loadedDataset.name);
				}
				if (!std::strcmp(bef.c_str(), INFO_TMAX)) {
					std::istringstream s(aftr);
					double tmax_d;
					s >> tmax_d;
					loadedDataset.Tmax = tmax_d;
					INFO("loaded tmax "<<loadedDataset.Tmax);
				}
				if (!std::strcmp(bef.c_str(), INFO_FORMAT)) {
					if (!std::strcmp(aftr.c_str(), MAP_FILE_FORMAT)) {
						INFO("using "<<MAP_FILE_FORMAT);
						file_format = map_file;
					} else if (!std::strcmp(aftr.c_str(), MAP_POINTS_CITY_POINTS_FORMAT)) {
						INFO("using "<<MAP_POINTS_CITY_POINTS_FORMAT);
						file_format = map_points_city_points;
					} else {
						ERROR(MAP_FILE_FORMAT<<" not specified or bad value "<<aftr.c_str());
						exit(1);
					}
				}

				if (file_format and !std::strcmp(bef.c_str(), INFO_MAP_FILE)) {
					std::string problem_file_path = getPath(filename);
					INFO("problem_file_path "<<problem_file_path)
					std::string blender_file = problem_file_path + PATH_SEPARATOR+aftr;
					INFO("blender_file "<<blender_file)
					mesh_objects = BlenderLoader::loadObjectsFromFile(blender_file);
					INFO("loaded blende objects");
				}

			}
			if (actual_loading == target_rewards) {
				INFO("loading target rewards")
				int delimiterPos = line.find(" ");
				std::string bef("");
				std::string aftr("");
				if (delimiterPos != std::string::npos) {
					bef = line.substr(0, delimiterPos);
					if (line.size() > delimiterPos + 1) {
						aftr = line.substr(delimiterPos + 1);
					}
					aftr = trim(aftr);
					bef = trim(bef);
					//INFO(" aftr "<<aftr<<" bef "<<bef)
				}

				double rew = std::stod(aftr);
				INFO("target "<<bef<<" rew "<<rew)
				target_reward_map.insert(std::pair<std::string, double>(bef, rew));
			}
		}
		loadedDataset.target_reward_map = target_reward_map;
		INFO("loaded "<<loaded_map_points<<" map points");

		INFO("and "<<target_nodes.size()<<" target locations");
	}
	loadedDataset.mesh_objects = mesh_objects;
	//exit(1);
	return loadedDataset;
}

std::vector<GraphNode<HeapPoint3D>> DatasetLoaderOBSTOP::parseInitialPositions(std::string string) {
	std::string delimiterPositions = "|";
	std::string delimiterXY = ";";
	std::vector<GraphNode<HeapPoint3D>> positions;
	std::vector<std::string> positionsArr;
	size_t pos = 0;
	std::string token;
	while ((pos = string.find(delimiterPositions)) != std::string::npos) {
		token = string.substr(0, pos);
		string.erase(0, pos + delimiterPositions.length());
		if (token.length() > 0) {
			positionsArr.push_back(token);
		}
	}
	if (string.length() > 0) {
		positionsArr.push_back(string);
	}
	for (int var = 0; var < positionsArr.size(); ++var) {
		std::string singlePosition = positionsArr[var];
		std::vector<std::string> xyPosition;
		INFO(singlePosition);
		size_t posIN = 0;
		std::string token;
		while ((posIN = singlePosition.find(delimiterXY)) != std::string::npos) {
			token = singlePosition.substr(0, posIN);
			singlePosition.erase(0, posIN + delimiterXY.length());

			if (token.length() > 0) {
				xyPosition.push_back(token);
			}
		}
		if (singlePosition.length() > 0) {
			xyPosition.push_back(singlePosition);
		}

		if (xyPosition.size() == 2) {
			float x;
			float y;
			std::sscanf(xyPosition[0].c_str(), "x=%f", &x);
			std::sscanf(xyPosition[1].c_str(), "y=%f", &y);
			INFO(x);
			INFO(y);
			HeapPoint3D data;
			data.x = x;
			data.y = y;
			data.z = 0;
			positions.push_back(GraphNode<HeapPoint3D>(data, 0, 0, 0));
		}
		for (int var2 = 0; var2 < xyPosition.size(); ++var2) {
			INFO(xyPosition[var2]);
		}
	}
	return positions;
}

std::string DatasetLoaderOBSTOP::trim(std::string str) {
	//INFO("trim")
	if (str.length() == 0) {
		return str;
	}
	size_t first = str.find_first_not_of(' ');
	size_t last = str.find_last_not_of(' ');
	if (first == std::string::npos || last == std::string::npos) {
		return std::string("");
	}
	//INFO_VAR(str)
	//INFO_VAR(str.length())
	//INFO_VAR(last)
	//INFO_VAR(first)
	return str.substr(first, (last - first + 1));
}

std::vector<std::string> DatasetLoaderOBSTOP::tokenize(std::string s, std::string delimiter) {
	std::vector<std::string> tokens;
	size_t pos = 0;
	std::string token;
	while ((pos = s.find(delimiter)) != std::string::npos) {
		token = s.substr(0, pos);
		tokens.push_back(token);
		//std::cout << token << std::endl;
		s.erase(0, pos + delimiter.length());
	}
	//std::cout << s << std::endl;
	return tokens;
}
