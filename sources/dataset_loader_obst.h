/*
 * DatasetLoaderGOP.h
 *
 *  Created on: 15. 3. 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_DATASETLOADER_OBSTOP_H_
#define SRC_DATASETLOADER_OBSTOP_H_
#include "heuristic_types.h"
#include "crl/logging.h"
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <cstdio>
#include <string>
#include <vector>
#include <cstring>
#include <algorithm>
#include <iostream>
#include "blender_loader.h"

class DatasetLoaderOBSTOP {
public:
	DatasetLoaderOBSTOP();
	virtual ~DatasetLoaderOBSTOP();
	static DatasetOBST_OP<HeapPoint3D> loadDatasetMapCityPoints(std::string filename);
	static DatasetOBST_OP_MeshObjects<HeapPoint3D> loadDatasetMapFile(std::string filename);
	static std::vector<GraphNode<HeapPoint3D>> parseInitialPositions(std::string string);
	static std::string trim(std::string str);
	static std::vector<std::string> tokenize(std::string s, std::string delimiter);

}
;

#endif /* SRC_DATASETLOADER_OBSTOP_H_ */
