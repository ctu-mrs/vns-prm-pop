/*
 * .h
 *
 *  Created on: 22. 3. 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_VNSGOP_VNSGOP_H_
#define SRC_VNSGOP_VNSGOP_H_

#include <vector>
#include <list>
#include "crl/logging.h"
#include "heuristic_types.h"
#include <cfloat>
#include "c_sort.h"
#include "coords.h"

#include <boost/foreach.hpp>
#include <crl/config.h>
#include <crl/alg/algorithm.h>
#include <crl/loader/text_format_loader.h>
#include "crl/random.h"
#include "crl/timerN.h"
#include "crl/perf_timer.h"
#include "crl/logging.h"
#include "crl/stringconversions.h"
#include "crl/loader/nodeloader.h"
#include "crl/alg/text_result_log.h"
#include "crl/file_utils.h"
#include "crl/assert.h"

#include "crl/gui/shapes.h"
#include "crl/gui/colormap.h"

#include "gui.h"
#include "canvasview_coords.h"
#include "math_common.h"
#include <unordered_set>
#include <vector>

#include <limits>

#include <algorithm>    // std::random_shuffle

#include "dataset_loader_obst.h"
#include "dataset_loader_obst.h"
#include "obst_op_loader.h"
#include "vns_obst_op_path.h"
#include "dijkstra.h"
#include "mesh_object.h"
//extern "C" {
//#include "triangle/triangle.h"
//}
#include "prm.h"

#define foreach BOOST_FOREACH
#define MIN_CHANGE_EPS 0.00001
#define DD " , "
#define DEBUG_DOP_TRY_OPERATIONS false
#define DEBUG_CHECK_CONSISTENCY false
#define DEBUG_DOP_IMRPOVE_OPERATIONS false
#define SAVE_ITER_PATHS false

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

using namespace crl;
using namespace crl::gui;

namespace op {

template<class T>
class VnsPrmOPSolver: public CAlgorithm {
	typedef CAlgorithm Base;
	typedef std::vector<int> IntVector;

public:
	VnsPrmOPSolver(CConfig& config, const std::string& problemFile);
	virtual ~VnsPrmOPSolver();
	//Tour find(DatasetOP dataset);
	//Tour find(std::vector<GraphNode> nodes, double budget, int startIndex_, int goalIndex_);

	//CAlgorithm functions
	void solve(void);
	std::string getRevision(void);
	static CConfig& getConfig(CConfig& config);
	static std::string getName(void) {
		return "vns_prm";
	}
	std::string getMethod(void) {
		return getName();
	}
	static PlanningState parse_planning_state(std::string planning_state_type_string);
	static MapFormat parse_map_format(std::string map_format_string);
protected:
	//CAlgorithm functions
	std::string getVersion(void);
	void load(void);
	void initialize(void);
	void after_init(void);
	void iterate(int step);
	void save(void);
	void release(void);
	void visualize(void);
	void defineResultLog(void);
	void fillResultRecord(int numIters, double length, int numItersLastImprovement, long timeLastImprovement,
			std::vector<ImprovementLogRecord> improvementLog);
	void set_borders(std::vector<HeapNode<HeapPoint3D> *> borders);
	CoordsVector getCoordPath(std::vector<IndexSOP> idPaths);

	void drawPath(int usleepTime = 0, VnsSopPath<T> * toShow = NULL);
	void drawVisibilityPath(int usleepTime, std::vector<HeapNode<T>> * toShow);
	std::vector<std::vector<double>> getTrajectoryPointVectors(std::vector<T> samples_traj);

	const double BORDER;
	const bool SAVE_RESULTS;
	const bool SAVE_INFO;
	const bool SAVE_SETTINGS;
	const bool SAVE_SAMPLED_PATH;
	const bool SAVE_TARGETS;

private:
	void calcClusterDistancesBounds();

	void save_path_during_optimization(int act_iter, double length, double reward);

	void checkConsistency(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS);
	void checkLengths(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, double newLength, std::string text =
			std::string(""));

	void shake(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int k,
			GraphNode<T> & chainGeneratingNeighAngEnd);
	bool randomLocalSearch(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int k,
			GraphNode<T> & chainGeneratingNeighAngEnd);

	//fill the city node clusters
	void fillCityNodes(OP_Prolem<HeapPoint3D> &problem);

	//methods for local search
	void onePointMove(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS);

	void fitDOPtoBudget(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS);

	void generateInitialSolution(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS);
	void optimize_path(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS);
	void optimize_all_node_pairs();
	void optimize_motion_between_targets(GraphNode<T> from, GraphNode<T> to, int add_samples);
	void update_roadmap_distances();

	void greedy_insertion(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS);

	bool insertRandom(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int num_changes);

	bool exchangeRandom(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int num_changes);

	bool check_try_operation(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int targetIDFrom,
			int targetIDTo, double lengthAfterMove, bool relocate);

	void pathInsert(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int k);
	void pathExchange(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int k);

	void save_prm_points(std::string filename);
	void fillImprovementRecord(ImprovementLogRecord& initialImprovementRecord, long timeLastImprovement, int iter);
	void createOPproblem();

	void save_prm_point_path(std::string filename, VnsSopPath<T> * toShow = NULL);
	//void save_prm_point_path_(std::string filename, std::vector<IndexSOP> &returnedPath);

	static int compareByDistanceFromStart();

	std::vector<MeshObject*> create_mesh_obstacles(std::vector<MapPoint> points_vec,
			std::vector<Obstacle> obstacles_vec);

	//just as saving id
	int path_var;
	GOPTYPE gopType;
	MapFormat map_format;

	//default params

	samples_distances_type nodesAllDistances;

	std::vector<std::vector<double>> nodesAllPairsSamplingDensity;
	std::vector<std::vector<double>> nodesAllPairsInRewardUsage;
	std::vector<double> cluster_rewards;
	int num_clusters;

	bool found_better_reward;

	PlanningState planning_state;
	std::string planning_state_type_str;

	//2d type
	PRM<T> * prm;
	std::vector<HeapNode<T>*> cities_nodes;
	std::vector<std::vector<GraphNode<T>>> nodesAllClusters;
	VnsSopPath<T> tourVNSGOPPath, finalTourVNSGOPPath;
	std::vector<std::vector<GraphNode<T>>> availableNodes;

	std::vector<std::vector<double>> clustersMinDistances;
	std::vector<std::vector<double>> clustersMaxDistances;

	MeshObject* mesh_robot;
	std::vector<MeshObject*> mesh_obstacles;

	int numAvailableNodes;
	double budget;
	int startIndex;
	int goalIndex;

	int initial_prm_size;
	int extension_prm_size;
	//int K;
	//int I;

	int oldgoalIndex;
	int oldstartIndex;
	int deleted_cluster_id;
	bool ids_originally_from_one;

	SamplingLimits sampling_limits;

	std::vector<MapPoint> map_points;
	std::vector<Obstacle> obstacles;
	std::vector<Obstacle> convex_regions;
	std::vector<Visibility> city2map_visibility_lists;
	std::vector<Visibility> map_visibility_lists;
	std::vector<Visibility> city2city_visibility_lists;
	int city_start_index;
	std::vector<HeapNode<HeapPoint3D> *> border_nodes;
	//std::vector<int> border;
	std::vector<HeapNode<HeapPoint2D>*> visibility_node_ids;

	double maximalRewardAll;
	double initial_reward;
	double optimize_initial_reward_ratio;
	double overbudget_optimize_ratio;
	int num_unreachable_nodes;

	std::vector<int> vnsVector;

	double maximal_calculation_time_sec;
	long maximal_calculation_time_MS;

	bool lower_bound_above_budget_then_skip;

	bool generate_examples_of_operations;
	bool set_greedy_initial_solution;
	bool useRVNS;
	int numIterations;
	int numIterationsUnimproved;

	double dubins_radius;
	double dubins_resolution;

	bool save_after_optimized_length;

	bool save_paths_during_optimization;

	bool stop;

	double maxOverGuessedTry;
	double avgsaved;
	double numTestedAvg;

	double fly_speed;
	double maximal_acceleration;
	bool radius_from_acc_speed;
	bool sample_by_time;
	double sample_time_s;
	double fly_altitude;
};

template<typename T>
VnsPrmOPSolver<T>::VnsPrmOPSolver(CConfig& config, const std::string& problemFile) :
		Base(config), SAVE_RESULTS(config.get<bool>("save-results")), SAVE_SETTINGS(config.get<bool>("save-settings")), BORDER(
				config.get<double>("canvas-border")), SAVE_INFO(config.get<bool>("save-info")), SAVE_TARGETS(
				config.get<bool>("save-targets")), SAVE_SAMPLED_PATH(config.get<bool>("save-sampled-path")) {

	//this->planning_state = state2dheading;
	this->output = config.get<std::string>("output");
	this->initial_prm_size = config.get<int>("initial-prm-size");
	this->extension_prm_size = config.get<int>("extension-prm-size");
	this->save_after_optimized_length = config.get<bool>("save-after-optimized-length");
	this->save_paths_during_optimization = config.get<bool>("save-paths-during-optimization");

	fly_speed = config.get<double>("fly-speed");
	maximal_acceleration = config.get<double>("maximal-acceleration");
	radius_from_acc_speed = config.get<bool>("radius-from-acc-speed");
	sample_by_time = config.get<bool>("sample-by-time");
	sample_time_s = config.get<double>("sample-time-s");
	fly_altitude = config.get<double>("fly-altitude");

	INFO_VAR(fly_speed);
	INFO_VAR(maximal_acceleration);
	INFO_VAR(radius_from_acc_speed);
	INFO_VAR(sample_by_time);
	INFO_VAR(sample_time_s);
	INFO_VAR(fly_altitude);

	std::string map_type_str = config.get<std::string>("map-type");
	INFO("map_type_str "<<map_type_str);
	this->map_format = this->parse_map_format(map_type_str);
	OP_Prolem<HeapPoint3D> problem;
	if (map_format == map_points_city_points) {
		INFO_GREEN("bef loaded useing getSOPDefinition")
		problem = OBSTOPLoader::getSOPDefinition(config, problemFile, true);
		INFO_GREEN("loaded useing getSOPDefinition")
		this->mesh_obstacles = problem.mesh_obstacles;
		this->mesh_robot = problem.mesh_robot;
		this->border_nodes = problem.border_nodes;

	} else if (map_format == map_file) {
		INFO_GREEN("bef loaded useing getSOPDefinitionMapFile")
		problem = OBSTOPLoader::getSOPDefinitionMapFile(config, problemFile, true);
		INFO_GREEN("loaded useing getSOPDefinitionMapFile")
		this->mesh_obstacles = problem.mesh_obstacles;
		this->mesh_robot = problem.mesh_robot;
		this->border_nodes = problem.border_nodes;

	} else {
		ERROR("unknown map_format");
		exit(1);
	}
	INFO("has SOP definition");
	this->name = config.get<std::string>("name");
	if (this->name.compare("no_name") == 0) {
		INFO("setting name to name from dataset " << problem.name);
		this->name = problem.name;
	}
	this->gopType = problem.gop_type;
	this->budget = problem.budget;
	if (config.get<double>("budget-override") != 0) {
		this->budget = config.get<double>("budget-override");
		INFO_RED("budget override from config by "<<this->budget);
	}
	this->startIndex = problem.startIndex;
	this->goalIndex = problem.goalIndex;
	this->maximalRewardAll = 0;

	planning_state_type_str = config.get<std::string>("planning-state");

	planning_state = VnsPrmOPSolver::parse_planning_state(planning_state_type_str);

	this->prm = new PRM<T>(config, planning_state);

	this->dubins_resolution = config.get<int>("dubins-resolution");
	this->dubins_radius = config.get<double>("dubins-radius");

	num_clusters = problem.samples.size();
	cluster_rewards.resize(num_clusters);
	fillCityNodes(problem);

	//this->nodesAllDistances = problem.distances;
	this->oldgoalIndex = problem.oldGoalIndex;
	this->oldstartIndex = problem.oldStartIndex;
	this->deleted_cluster_id = problem.deleted_cluster_id;
	this->ids_originally_from_one = problem.ids_originally_from_one;
	this->obstacles = problem.obstacles;
	this->map_points = problem.map_points;
	this->convex_regions = problem.convex_regions;
	this->city2map_visibility_lists = problem.city2map_visibility_lists;
	this->map_visibility_lists = problem.map_visibility_lists;
	this->city2city_visibility_lists = problem.city2city_visibility_lists;
	this->city_start_index = 0;

	this->optimize_initial_reward_ratio = 0.95;
	this->overbudget_optimize_ratio = 1.2;

	INFO_GREEN("maximalRewardAll "<<this->maximalRewardAll);

	this->maxOverGuessedTry = 0;
	this->avgsaved = NAN;
	this->numTestedAvg = 0;
	this->stop = false;
	this->path_var = 0;
	this->numAvailableNodes = 0;
	this->initial_reward = 0;
	this->useRVNS = config.get<bool>("use-rvns");
	this->numIterations = config.get<int>("num-iterations");
	this->numIterationsUnimproved = config.get<int>("num-iterations-unimproved");
	this->maximal_calculation_time_sec = config.get<int>("maximal-calculation-time-sec");
	this->maximal_calculation_time_MS = maximal_calculation_time_sec * 1000.0;
	this->lower_bound_above_budget_then_skip = config.get<bool>("lower-bound-above-budget-then-skip");
	INFO_VAR(lower_bound_above_budget_then_skip);
	this->set_greedy_initial_solution = config.get<bool>("initial-greedy-solution");
	this->generate_examples_of_operations = config.get<bool>("generate-examples-of-operations");

	this->found_better_reward = false;

}

template<typename T>
VnsPrmOPSolver<T>::~VnsPrmOPSolver() {
	delete this->prm;
}

template<typename T>
void VnsPrmOPSolver<T>::solve() {
	INFO("solve begin")
	Base::solve();
	INFO("solve end")
}

template<typename T>
std::string VnsPrmOPSolver<T>::getVersion(void) {
	return "OPFourPhaseHeuristic 0.1";
}

template<typename T>
void VnsPrmOPSolver<T>::initialize(void) {

}

template<typename T>
std::string VnsPrmOPSolver<T>::getRevision(void) {
	return "$Id$";
}

template<typename T>
void VnsPrmOPSolver<T>::after_init(void) {

}

template<typename T>
void VnsPrmOPSolver<T>::release(void) {

}

template<typename T>
void VnsPrmOPSolver<T>::visualize(void) {
}

template<typename T>
CConfig & VnsPrmOPSolver<T>::getConfig(CConfig & config) {
	return config;
}

template<typename T>
PlanningState VnsPrmOPSolver<T>::parse_planning_state(std::string planning_state_type_string) {
	PlanningState readed_planning_state;
	planning_state_type_string = trim(planning_state_type_string);
	if (planning_state_type_string.compare("2d") == 0) {
		readed_planning_state = state2d;
	} else if (planning_state_type_string.compare("dubins2d") == 0) {
		readed_planning_state = state2dheading;
	} else if (planning_state_type_string.compare("3d") == 0) {
		readed_planning_state = state3d;
	} else {
		ERROR("unknown planning_state_type type "<<planning_state_type_string);
		exit(1);
	}
	return readed_planning_state;
}

template<typename T>
MapFormat VnsPrmOPSolver<T>::parse_map_format(std::string map_format_string) {
	MapFormat map_format;
	INFO("parse_map_format !!!!!")
	map_format_string = trim(map_format_string);
	INFO("map_format_string "<<map_format_string)
	if (map_format_string.compare("MAP_POINTS_CITY_POINTS") == 0) {
		map_format = map_points_city_points;
		INFO("map_format_string map_points_city_points")
	} else if (map_format_string.compare("MAP_FILE") == 0) {
		map_format = map_file;
		INFO("map_format_string map_file")
	} else {
		ERROR("unknown planning_state_type type "<<map_format_string);
		exit(1);
	}
	return map_format;
}

template<typename T>
void VnsPrmOPSolver<T>::iterate(int iter) {

	//CShape black_line("black", "white", 1, 0);
	//if (canvas) {
	//	drawObstacle(*canvas, mesh_obstacle, map_points, &black_line);
	//}

	nodesAllDistances.resize(num_clusters);
	clustersMinDistances.resize(num_clusters);
	clustersMaxDistances.resize(num_clusters);
	nodesAllPairsSamplingDensity.resize(num_clusters);
	nodesAllPairsInRewardUsage.resize(num_clusters);
	for (int fromid = 0; fromid < num_clusters; ++fromid) {
		nodesAllDistances[fromid].resize(num_clusters);
		clustersMinDistances[fromid].resize(num_clusters);
		clustersMaxDistances[fromid].resize(num_clusters);
		nodesAllPairsSamplingDensity[fromid].resize(num_clusters);
		nodesAllPairsInRewardUsage[fromid].resize(num_clusters);
		for (int toid = 0; toid < nodesAllDistances[fromid].size(); ++toid) {
			if (planning_state == state2d || planning_state == state3d) {
				nodesAllDistances[fromid][toid].resize(1);
				nodesAllDistances[fromid][toid][0].resize(1);
			} else if (planning_state == state2dheading) {
				nodesAllDistances[fromid][toid].resize(dubins_resolution);
				for (int head_id = 0; head_id < dubins_resolution; ++head_id) {
					nodesAllDistances[fromid][toid][head_id].resize(dubins_resolution);
				}
			}
			nodesAllPairsSamplingDensity[fromid][toid] = 0.0001;
			nodesAllPairsInRewardUsage[fromid][toid] = 0;
		}
	}

	//firstly select points in eclipse
	INFO("start timer")
	crl::CTimerN testTouring;

	INFO("num obstacels="<<mesh_obstacles.size())
	//exit(0);

	prm->set_borders(border_nodes);
	this->set_borders(border_nodes);
	prm->set_gui(canvas);

	INFO("resetRealClock")
	testTouring.resetRealClock();
	prm->create_initial_graph(this->mesh_obstacles, mesh_robot, cities_nodes, initial_prm_size);
	this->update_roadmap_distances();

	//sleep(100);
	INFO("num-iterations " << numIterations);
	INFO("num-iterations-unimproved " << numIterationsUnimproved);

	VnsSopPath<T>::setSamplesDistances(&nodesAllClusters, &nodesAllDistances, &clustersMinDistances,
			&clustersMaxDistances);
	INFO("set prm bef");
	VnsSopPath<T>::setPRM(prm);
	INFO("set prm aft");

	if (config.get<bool>("create-ilp-op-problem")) {
		INFO("creating ilp of problem")
		this->createOPproblem();
	}


	this->availableNodes = this->nodesAllClusters;
	INFO("remove start goal from available nodes "<<availableNodes.size());
	if (startIndex != goalIndex) {
		this->availableNodes.erase(availableNodes.begin() + MAX(startIndex, goalIndex));
		this->availableNodes.erase(availableNodes.begin() + MIN(startIndex, goalIndex));
	} else {
		this->availableNodes.erase(availableNodes.begin() + startIndex);
	}
	this->numAvailableNodes = availableNodes.size();
	INFO("we have " << availableNodes.size() << " nodes");

	INFO_VAR(startIndex);
	INFO_VAR(goalIndex);
	INFO("print clusters before iterate");
	//SOPLoader::printAllClustersNodes(nodesAllClusters);

	INFO("initialize SOP");
	double initial_path_length = M;
	tourVNSGOPPath = VnsSopPath<T>(startIndex, goalIndex, gopType);
	initial_path_length = tourVNSGOPPath.getPathLength();
	INFO("tourVNSGOPPath2d created with length "<<tourVNSGOPPath.getPathLength());

	while (initial_path_length > budget) {
		INFO_RED("can not find initial solution");
		INFO("add additional "<<initial_prm_size<<" points")
		prm->add_uniform_points(initial_prm_size);
		prm->calculate_added_points();
		update_roadmap_distances();
		tourVNSGOPPath = VnsSopPath<T>(startIndex, goalIndex, gopType);
		initial_path_length = tourVNSGOPPath.getPathLength();
		INFO("tourVNSGOPPath2d created with length "<<tourVNSGOPPath.getPathLength());
	}
	for (int var = 0; var < availableNodes.size(); ++var) {
		vnsVector.push_back(availableNodes[var][0].cluster_id);
	}
	drawPath(2000);
	//sleep(10);
	path_var = 1;
	if (set_greedy_initial_solution) {
		//INFO("generateInitialSolution beg");
		generateInitialSolution(tourVNSGOPPath, vnsVector);
		//INFO("generateInitialSolution end");
	}
	this->initial_reward = tourVNSGOPPath.getReward();

	//for example generation of neighborhood operations
	int numItersLastImprovement = 1;
	long timeLastImprovement = testTouring.getRTimeMS();

	std::vector<ImprovementLogRecord> improvementLog;

	ImprovementLogRecord initialImprovementRecord;
	fillImprovementRecord(initialImprovementRecord, timeLastImprovement, numItersLastImprovement);
	improvementLog.push_back(initialImprovementRecord);

	int act_iter = 0;
	/*
	 std::stringstream prm_points_filename;
	 prm_points_filename << "improvement_iter_" << act_iter << "_prm_points.txt";
	 save_prm_points(prm_points_filename.str());
	 std::stringstream prm_points_path_filename;
	 prm_points_path_filename << "improvement_iter_" << act_iter << "_prm_point_path.txt";
	 save_prm_point_path(prm_points_path_filename.str());
	 */

	INFO_GREEN(
			"initial solution with reward "<<tourVNSGOPPath.getReward()<<", length "<<tourVNSGOPPath.getPathLength()<<" and budget "<<budget<<" at time "<<testTouring.getRTimeMS()<<" ms");

	stop = false;
	int initialNDepth = 1;
	int maximalNDepth = 2;
	while (!stop) {
		INFO("act_iter "<<act_iter);
		act_iter++;
		//double lengthBeforePRMOptimization = tourVNSGOPPath.getPathLength();

		//INFO("itteration "<<numIttertation);
		if (act_iter % 2 == 0) {
			INFO(
					"itteration " << act_iter << " (last improved "<<numItersLastImprovement<<" with best reward " << tourVNSGOPPath.getReward() << ", length " << tourVNSGOPPath.getPathLength() << " and budget " << budget << " at time " << testTouring.getRTimeMS() << " ms");
			//INFO("numItersLastImprovement "<<numItersLastImprovement)
		}
		if (act_iter >= numIterations) {
			INFO("stop after maximal number of iterattion " << numIterations);
			stop = true;
		}
		if (act_iter - numItersLastImprovement >= numIterationsUnimproved) {
			INFO("stop after maximal number of iterattion without improvement " << numIterationsUnimproved);
			stop = true;
		}
		int k = initialNDepth;
		//INFO("k "<<k);
		while (k <= maximalNDepth) {
			if (testTouring.getRTimeMS() >= maximal_calculation_time_MS) {
				INFO(
						"stop at " << testTouring.getRTimeMS() << " after maximal number of misiliseconds " << maximal_calculation_time_MS << " obtained from " << maximal_calculation_time_sec << " maximal seconds");
				stop = true;
			}

			VnsSopPath<T> actualVNSGOPPath = tourVNSGOPPath;
			std::vector<int> actualVNS = vnsVector;
			double rewardBefore = actualVNSGOPPath.getReward();
			double lengthBefore = actualVNSGOPPath.getPathLength();

			GraphNode<T> localNeighChain = GraphNode<T>();
			shake(actualVNSGOPPath, actualVNS, k, localNeighChain);
			randomLocalSearch(actualVNSGOPPath, actualVNS, k, localNeighChain);

			double newReward = actualVNSGOPPath.getReward();
			double newLength = actualVNSGOPPath.getPathLength(localNeighChain);

			if (newReward > rewardBefore || (newReward == rewardBefore && newLength < lengthBefore)) {
				INFO_GREEN(
						"improved to reward "<<newReward<<" with length "<<newLength<<" and budget "<<budget<<" at time "<<testTouring.getRTimeMS()<<" ms");
				timeLastImprovement = testTouring.getRTimeMS();
				numItersLastImprovement = act_iter;

				tourVNSGOPPath = actualVNSGOPPath;
				vnsVector = actualVNS;

				checkLengths(tourVNSGOPPath, vnsVector, newLength, "in badly cal new solution");

				ImprovementLogRecord newImprovementRecord;
				fillImprovementRecord(newImprovementRecord, timeLastImprovement, act_iter);
				improvementLog.push_back(newImprovementRecord);

				//save_path_during_optimization(act_iter, tourVNSGOPPath.getPathLength(), tourVNSGOPPath.getReward());

				drawPath(2000, &actualVNSGOPPath);

				k = initialNDepth;
			} else {
				k++;
			}

		}

		optimize_all_node_pairs();

		save_path_during_optimization(act_iter, tourVNSGOPPath.getPathLength(), tourVNSGOPPath.getReward());

	}

	tSolve.stop();

	finalTourVNSGOPPath = tourVNSGOPPath;
	INFO("fillResultRecord");
	fillResultRecord(act_iter, tourVNSGOPPath.getPathLength(), numItersLastImprovement, timeLastImprovement,
			improvementLog);
	INFO("write result log");
	INFO_GREEN(
			"found tour with reward "<<finalTourVNSGOPPath.getReward()<<" and length "<<finalTourVNSGOPPath.getPathLength()<<" out of "<<budget<<" budget");

	resultLog << result::endrec;
	INFO("end");
}

template<typename T>
void VnsPrmOPSolver<T>::save_path_during_optimization(int act_iter, double length, double reward) {
	if (save_paths_during_optimization) {
		INFO_GREEN("saving path during optimization");
		std::stringstream prm_points_filename;
		prm_points_filename << "improvement_iter_" << act_iter << "_prm_points.txt";
		save_prm_points(prm_points_filename.str());

		std::stringstream prm_points_path_filename;
		prm_points_path_filename << "improvement_iter_" << act_iter << "_prm_point_path.txt";
		VnsSopPath<T> *toShow = NULL;
		save_prm_point_path(prm_points_path_filename.str(), toShow);

		std::stringstream path_params_filename;
		path_params_filename << "improvement_iter_" << act_iter << "_path_params.txt";
		std::string dir = output;
		std::string file = getOutputIterPath(path_params_filename.str(), dir);
		assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream out(file.c_str());
		assert_io(out.good(), "Cannot create path '" + file + "'");
		std::string delimiter = ", ";
		if (out.is_open()) {
			out << reward << delimiter << length;
			out.close();
		} else {
			std::cerr << "Cannot open " << file << std::endl;
		}
	}
}

template<typename T>
void VnsPrmOPSolver<T>::calcClusterDistancesBounds() {
	//INFO("calc lower and upper bound on cluster distances - begin")
	int num_unvisitable_clusters = 0;

	for (int cl1 = 0; cl1 < nodesAllDistances.size(); ++cl1) {
		for (int cl2 = 0; cl2 < nodesAllDistances[cl1].size(); ++cl2) {
			double minDistance = M;
			double maxDistance = -M;
			int cl1NodesNum = nodesAllDistances[cl1][cl2].size();
			bool is_path_between_clusters = false;
			for (int cl1node = 0; cl1node < cl1NodesNum; ++cl1node) {
				int cl2NodesNum = nodesAllDistances[cl1][cl2][cl1node].size();

				for (int cl2node = 0; cl2node < cl2NodesNum; ++cl2node) {
					if (nodesAllDistances[cl1][cl2][cl1node][cl2node] != M) {
						is_path_between_clusters = true;

					}
					if (nodesAllDistances[cl1][cl2][cl1node][cl2node] < minDistance) {
						minDistance = nodesAllDistances[cl1][cl2][cl1node][cl2node];
					}

					if (nodesAllDistances[cl1][cl2][cl1node][cl2node] > maxDistance) {
						maxDistance = nodesAllDistances[cl1][cl2][cl1node][cl2node];
					}
				}
			}
			if (!is_path_between_clusters) {
				num_unvisitable_clusters++;
				//INFO_RED("no path between targets "<<cl1<<" and "<<cl2)
			}
			clustersMinDistances[cl1][cl2] = minDistance;
			clustersMaxDistances[cl1][cl2] = maxDistance;
		}
	}
	num_unreachable_nodes = num_unvisitable_clusters;
	if (num_unreachable_nodes > 0) {
		INFO_VAR_RED(num_unreachable_nodes);
	}
	//INFO("calc lower and upper bound on cluster distances - end")
}

template<typename T>
void VnsPrmOPSolver<T>::optimize_all_node_pairs() {
	//INFO("optimize_all_node_pairs begin");
	double num_to_add_samples = extension_prm_size;
	//if(found_better_reward){
	//	num_to_add_samples = 10*num_to_add_samples;
	//}
	if (num_to_add_samples > 0) {
		double sum_optimize_reward = 0;
		double sum_density = 0;
		for (int from_node = 0; from_node < num_clusters; ++from_node) {
			for (int to_node = 0; to_node < num_clusters; ++to_node) {
				if (from_node != to_node) {
					if (isnan(nodesAllPairsInRewardUsage[from_node][to_node])) {
						INFO_VAR_RED(nodesAllPairsInRewardUsage[from_node][to_node]);
						exit(1);
					}
					if (isnan(nodesAllPairsSamplingDensity[from_node][to_node])) {
						INFO_VAR_RED(nodesAllPairsSamplingDensity[from_node][to_node]);
						exit(1);
					}
					sum_optimize_reward += nodesAllPairsInRewardUsage[from_node][to_node];
					sum_density += nodesAllPairsSamplingDensity[from_node][to_node];
				}
			}
		}

		//INFO_VAR_RED(sum_optimize_reward)
		//INFO_VAR_RED(sum_density)

		double add_samples_sum = 0;

		std::vector<std::vector<double>> relativeRewards = nodesAllPairsInRewardUsage;
		std::vector<std::vector<double>> relativeDensity = nodesAllPairsSamplingDensity;
		int num_pairs = 0;
		for (int from_node = 0; from_node < num_clusters; ++from_node) {
			for (int to_node = from_node + 1; to_node < num_clusters; ++to_node) {
				relativeRewards[from_node][to_node] = nodesAllPairsInRewardUsage[from_node][to_node]
						/ sum_optimize_reward + nodesAllPairsInRewardUsage[to_node][from_node] / sum_optimize_reward;

				relativeDensity[from_node][to_node] = nodesAllPairsSamplingDensity[from_node][to_node] / sum_density
						+ nodesAllPairsSamplingDensity[to_node][from_node] / sum_density;
				num_pairs++;
				add_samples_sum += relativeRewards[from_node][to_node] / relativeDensity[from_node][to_node];
			}
		}
		INFO("average density:"<<sum_density/num_pairs)
		INFO("average rewards:"<<sum_optimize_reward/num_pairs)
		//INFO(add_samples_sum);

		if (num_unreachable_nodes > 0) {
			INFO("some ("<<num_unreachable_nodes<<") unreachable nodes - add "<<num_to_add_samples<<" uniform samples")
			prm->add_uniform_points(num_to_add_samples);
		}

		double sum_added_samples = 0;

		for (int from_node = 0; from_node < num_clusters; ++from_node) {
			for (int to_node = from_node + 1; to_node < num_clusters; ++to_node) {
				//INFO("sample " << var << " id " << samples[var][0].cluster_id << " pos " << samples[var][0].x << " " << samples[var][0].y );

				if (from_node != to_node) {
					//sum_optimize_what += ceil((1.0 / nodesAllPairsSamplingDensity[from_node][to_node] * 1000) * nodesAllPairsInRewardUsage[from_node][to_node]/ (maximalRewardAll * numAvailableNodes * numAvailableNodes));
					//int add_samples = (nodesAllPairsInRewardUsage[from_node][to_node] / sum_optimize_reward) * (1.0 / (nodesAllPairsSamplingDensity[from_node][to_node] / sum_density));

					double add_samples = ceil(
							num_to_add_samples
									* (relativeRewards[from_node][to_node] / relativeDensity[from_node][to_node])
									/ add_samples_sum);

					//INFO("add_samples "<<add_samples<<" between "<<from_node<<" "<<to_node);
					sum_added_samples += add_samples;

					GraphNode<T> opt_from = nodesAllClusters[from_node][0];
					GraphNode<T> opt_to = nodesAllClusters[to_node][0];
					optimize_motion_between_targets(opt_from, opt_to, add_samples);
					nodesAllPairsInRewardUsage[to_node][from_node] = 0;
					nodesAllPairsInRewardUsage[from_node][to_node] = 0;

				}
			}
		}
		INFO_RED("sum_added_samples "<<sum_added_samples);
		//INFO("calculate_added_points");
		if (sum_added_samples > 0) {

			prm->calculate_added_points();
			this->update_roadmap_distances();
			tourVNSGOPPath.update();
			drawPath();

		}
	}
	this->found_better_reward = false;
	//INFO("optimize_all_node_pairs end");
}

template<typename T>
void VnsPrmOPSolver<T>::update_roadmap_distances() {
	//INFO("update_roadmap_distances begin");
	for (int fromcl = 0; fromcl < num_clusters; ++fromcl) {
		//INFO_VAR(fromcl);
		std::map<int, std::pair<int, int>> node_ids_poisitions;

		for (int from_node = 0; from_node < nodesAllClusters[fromcl].size(); ++from_node) {
			//INFO_VAR(from_node);
			int fromid = nodesAllClusters[fromcl][from_node].id;
			//INFO_VAR(fromid);
			node_ids_poisitions[fromid] = std::pair<int, int>(fromcl, from_node);
			std::vector<int> to_ids;
			//std::stringstream ss;
			for (int tocl = 0; tocl < num_clusters; ++tocl) {
				for (int to_id = 0; to_id < nodesAllClusters[tocl].size(); ++to_id) {
					int toid = nodesAllClusters[tocl][to_id].id;
					to_ids.push_back(toid);
					//ss <<toid<<" ";
					node_ids_poisitions[toid] = std::pair<int, int>(tocl, to_id);
				}
			}

			//INFO("to_ids:"<<ss.str())
			std::vector<plan_with_length<T>> found_paths = prm->plan(fromid, to_ids);
			for (int path_id = 0; path_id < found_paths.size(); ++path_id) {
				int to_node_id = found_paths[path_id].to_id;
				int from_node_id = found_paths[path_id].from_id;
				std::pair<int, int> to_pair = node_ids_poisitions[to_node_id];
				std::pair<int, int> from_pair = node_ids_poisitions[from_node_id];
				//INFO("find from "<<from_pair.first<<" to "<<to_pair.first<<" len "<< found_paths[path_id].lenght)
				nodesAllDistances[from_pair.first][to_pair.first][from_pair.second][to_pair.second] =
						found_paths[path_id].lenght;
			}
		}

	}
	this->calcClusterDistancesBounds();
	//INFO("update_roadmap_distances end");
	//exit(1);
}

template<typename T>
void VnsPrmOPSolver<T>::load(void) {
// nothing to load, structures are passed to the constructor
	//INFO("load");
	int n = map_points.size();
	if (canvas) {
		CoordsVector points;
		//INFO("BORDER " << BORDER);
		for (int mid = 0; mid < map_points.size(); ++mid) {
			Coords coord_up(map_points[mid].x + BORDER, map_points[mid].y + BORDER);
			Coords coord_down(map_points[mid].x - BORDER, map_points[mid].y - BORDER);
			points.push_back(coord_up);
			points.push_back(coord_down);
			INFO("x:"<<map_points[mid].x<< " y:"<<map_points[mid].y)
		}

		*canvas << canvas::AREA;
		INFO("draw points");
		for (int var = 0; var < points.size(); ++var) {
			*canvas << points[var];
			//INFO("x:"<<points[var].x<< " y:"<<points[var].y)
		}
		INFO("set to canvas");
		*canvas << canvas::END;

		//print border
		/*
		 if (this->border_nodes.size() > 0) {
		 *canvas << "border" << canvas::LINESTRING;
		 for (int var = 0; var < border_nodes.size(); ++var) {
		 INFO("border point "<< border_nodes[var]->data.x <<" "<< border_nodes[var]->data.y);
		 *canvas << border_nodes[var]->data.x << border_nodes[var]->data.y;
		 }
		 *canvas << border_nodes[0]->data.x << border_nodes[0]->data.y;
		 *canvas << canvas::END;
		 }
		 */

		//print obstacles
		CShape redLine("red", "red", 1, 0);
		*canvas << "obstacles";
		for (int obstid = 0; obstid < obstacles.size(); ++obstid) {
			*canvas << canvas::POLYGON << redLine;
			for (int verid = 0; verid < obstacles[obstid].point_indexes.size(); ++verid) {
				//INFO(	"obstacle point "<< map_points[obstacles[obstid].point_indexes[verid]].x <<" "<< map_points[obstacles[obstid].point_indexes[verid]].y);
				*canvas << map_points[obstacles[obstid].point_indexes[verid]].x
						<< map_points[obstacles[obstid].point_indexes[verid]].y;
			}
			*canvas << canvas::END;
		}

		//*canvas << map_points[border[0]].x << map_points[border[0]].y;
		if (config.get<bool>("draw-stations")) {
			INFO("draw stations");
			std::string pallete = config.get<std::string>("draw-targets-reward-palette");
			if (config.get<bool>("draw-targets-reward") and isFile(pallete)) {
				INFO("draw-targets-reward from pallete");
				CColorMap map;
				map.load(pallete);
				double minReward = DBL_MAX;
				double maxReward = -DBL_MAX;
				for (int var = 0; var < num_clusters; ++var) {
					if (cluster_rewards[var] < minReward) {
						minReward = cluster_rewards[var];
					}
					if (cluster_rewards[var] > maxReward) {
						maxReward = cluster_rewards[var];
					}
				}

				if (minReward == maxReward) {
					minReward = 0.99 * maxReward;
				}
				INFO_VAR(minReward);
				INFO_VAR(maxReward);
				map.setRange(minReward, maxReward);
				*canvas << "targets" << CShape(config.get<std::string>("draw-shape-targets")) << canvas::POINT;

				for (int clusterID = 0; clusterID < num_clusters; ++clusterID) {
					for (int clusterNodeID = 0; clusterNodeID < nodesAllClusters[clusterID].size(); ++clusterNodeID) {
						Coords cord(nodesAllClusters[clusterID][clusterNodeID].data.x,
								nodesAllClusters[clusterID][clusterNodeID].data.y);
						*canvas << canvas::FILL_COLOR
								<< map.getColor((double) nodesAllClusters[clusterID][clusterNodeID].reward) << cord;
					}
				}

			} else {
				INFO("draw stations withou pallete");
				*canvas << "stations" << CShape(config.get<std::string>("draw-shape-stations")) << canvas::POINT;

				for (int clusterID = 0; clusterID < nodesAllClusters.size(); ++clusterID) {
					for (int clusterNodeID = 0; clusterNodeID < nodesAllClusters[clusterID].size(); ++clusterNodeID) {
						Coords cord(nodesAllClusters[clusterID][clusterNodeID].data.x,
								nodesAllClusters[clusterID][clusterNodeID].data.y);
						*canvas << cord << canvas::END;
					}
				}
			}
		}

		canvas->redraw();

		INFO("exit load");
		//exit(1);
	} else {
		INFO("no canvas");
	}			//end canvas
}

template<typename T>
void VnsPrmOPSolver<T>::generateInitialSolution(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS) {
	INFO("generateInitialSolution() begin");
	greedy_insertion(actualVNSGOPPath, actualVNS);
	INFO("reward after insertion " << actualVNSGOPPath.getReward());
	INFO("num targets " << actualVNSGOPPath.getNumTargets());
	INFO("generateInitialSolution() end");
}

template<typename T>
void VnsPrmOPSolver<T>::checkLengths(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, double newLength,
		std::string text) {
	double calculated_length = actualVNSGOPPath.getPathLengthCalculate();
	if (calculated_length > budget || newLength > budget || calculated_length - newLength > 0.01) {
		ERROR("badly calculated length in new solution");
		INFO_VAR(budget);
		INFO_VAR(newLength);
		INFO_VAR(calculated_length);

		exit(1);
	}
}

template<typename T>
void VnsPrmOPSolver<T>::greedy_insertion(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS) {
	INFO("greedy_insertion begin");

	int waitTimeus = 1000;
	bool somethingAdded = true;

	while (somethingAdded) {
		//INFO("inserting");
		somethingAdded = false;
		double minimalAddDistPerReward = DBL_MAX;
		double actualLength = actualVNSGOPPath.getPathLength();
		double newLengthAfterAdd = actualLength;
		int idAvNodeMinimal = -1;
		int idTourNodeMinimal = -1;

		int dopSize = actualVNSGOPPath.getNumTargets();
		//INFO_VAR(dopSize);
		for (int idAvNode = dopSize; idAvNode < actualVNS.size(); ++idAvNode) {
			//try to add between start ang goal
			int testingNodeID = actualVNS[idAvNode];
			double testingNodeReward = cluster_rewards[testingNodeID];
			for (int idTourNode = 0; idTourNode < actualVNSGOPPath.getSize(); ++idTourNode) {
				//INFO("try to add idAvNode "<<idAvNode<<" with reward "<<testingNodeReward<<" to position "<<idTourNode<<" clusterid "<<testingNodeID);
				double newDistance = actualVNSGOPPath.tryToAdd(testingNodeID, idTourNode);
				double additionalDistance = newDistance - actualLength;
				double additionalDistPerReward = additionalDistance / testingNodeReward;
				//INFO("distance "<<newDistance);
				if (additionalDistPerReward < minimalAddDistPerReward && newDistance <= budget) {
					//INFO("additionalDistPerReward bettew "<<additionalDistPerReward);
					minimalAddDistPerReward = additionalDistPerReward;
					newLengthAfterAdd = newDistance;
					idAvNodeMinimal = idAvNode;
					idTourNodeMinimal = idTourNode;
				}
			}
		}

		if (idAvNodeMinimal >= 0 && idTourNodeMinimal >= 0 && newLengthAfterAdd <= budget) {
			somethingAdded = true;
			//INFO("add idAvNode "<<idAvNodeMinimal<<" to position "<<idTourNodeMinimal);

			int insertingID = actualVNS[idAvNodeMinimal];
			actualVNSGOPPath.addPoint(insertingID, idTourNodeMinimal);
			//INFO("point added with insertingID "<<insertingID<<" new len sholud be "<<newLengthAfterAdd);
			double newlenis = actualVNSGOPPath.getPathLength();
			//INFO("new len is "<<newlenis);
			//savePaths(&actualVNSGOPPath);
			//INFO("improveNeighLocations after insert");
			//improveNeighLocations(actualVNSGOPPath, actualVNS, 1);
			//INFO("improveNeighLocations after insert end");

			if (idAvNodeMinimal > idTourNodeMinimal) {
				//INFO("erase1");
				actualVNS.erase(actualVNS.begin() + idAvNodeMinimal);
				actualVNS.insert(actualVNS.begin() + idTourNodeMinimal, insertingID);
			} else {
				//INFO("erase2");
				actualVNS.insert(actualVNS.begin() + idTourNodeMinimal, insertingID);
				actualVNS.erase(actualVNS.begin() + idAvNodeMinimal);
			}
			//INFO("check consistency");
			checkConsistency(actualVNSGOPPath, actualVNS);
			//INFO("draw path");
			//drawPath<T>(waitTimeus);
			//INFO("draw path end");
		}

	}

	checkConsistency(actualVNSGOPPath, actualVNS);
	INFO("insertion end");
}

template<typename T>
void VnsPrmOPSolver<T>::shake(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int k,
		GraphNode<T> & chainGeneratingNeighAngEnd) {
//INFO("shake "<<k);
	if (k == 1) {
		pathInsert(actualVNSGOPPath, actualVNS, 1);
	} else if (k == 2) {
		pathExchange(actualVNSGOPPath, actualVNS, 1);
	}
	if ( DEBUG_DOP_TRY_OPERATIONS) {
		INFO_VAR(DEBUG_DOP_TRY_OPERATIONS);
		checkConsistency(actualVNSGOPPath, actualVNS);
	}
//INFO("shake done "<<k);
}

template<typename T>
bool VnsPrmOPSolver<T>::randomLocalSearch(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int k,
		GraphNode<T> & chainGeneratingNeighAngEnd) {
	int numIters = numAvailableNodes * numAvailableNodes;
	if (k == 1) {
		insertRandom(actualVNSGOPPath, actualVNS, numIters);
	} else if (k == 2) {
		exchangeRandom(actualVNSGOPPath, actualVNS, numIters);
	}
	if ( DEBUG_DOP_TRY_OPERATIONS) {
		INFO_VAR(DEBUG_DOP_TRY_OPERATIONS);
		checkConsistency(actualVNSGOPPath, actualVNS);
	}

	return true;
}

template<typename T>
void VnsPrmOPSolver<T>::fitDOPtoBudget(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS) {
	//remove nodes over budget
	while (actualVNSGOPPath.getPathLength() > budget) {
		actualVNSGOPPath.removePoint(actualVNSGOPPath.getNumTargets() - 1);
	}
	bool canAdd = true;

	//add nodes to fill budget
	while (canAdd) {
		canAdd = false;
		int idToAdd = actualVNSGOPPath.getNumTargets();
		if (idToAdd < actualVNS.size()) {
			int testingNode = actualVNS[idToAdd];
			double testAdd = actualVNSGOPPath.tryToAdd(testingNode, idToAdd);
			if (testAdd <= budget) {
				actualVNSGOPPath.addPoint(testingNode, idToAdd);
				canAdd = true;
			}
		}
	}
}

template<typename T>
void VnsPrmOPSolver<T>::onePointMove(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS) {
	INFO("onePointMove() begin");

	int bestTargetIDFrom = -1;
	int bestTargetIDTo = -1;
	double actualPathLength = actualVNSGOPPath.getPathLength();
	double actualPathReward = actualVNSGOPPath.getReward();
	double minimalLength = actualPathLength;
	double maximalReward = actualPathReward;

	for (int targetIDFrom = actualVNSGOPPath.getNumTargets(); targetIDFrom < actualVNS.size(); ++targetIDFrom) {
		int testingRelocateID = actualVNS[targetIDFrom];
		double testingRelocateReward = cluster_rewards[testingRelocateID];

		for (int targetIDTo = 0; targetIDTo <= actualVNSGOPPath.getNumTargets(); ++targetIDTo) {
			if (targetIDFrom != targetIDTo) {
				double addedLength = 0;
				double addedReward = 0;
				if (targetIDTo <= actualVNSGOPPath.getNumTargets()) {
					addedLength = actualVNSGOPPath.tryToAdd(testingRelocateID, targetIDTo);
					addedReward = testingRelocateReward;
				}

				double rewardAfter = actualPathReward + addedReward;
				double lengthAfter = actualPathLength + addedLength;
				/*
				 if (lengthAfter <= budget) {
				 INFO("rewardChange "<<(addedReward - removedReward));
				 INFO("lengthChange "<<(addedLength - removedLength));
				 }
				 */
				if (lengthAfter <= budget
						&& (rewardAfter > maximalReward || (rewardAfter == maximalReward && lengthAfter < minimalLength))) {
					bestTargetIDFrom = targetIDFrom;
					bestTargetIDTo = targetIDTo;
					maximalReward = rewardAfter;
					minimalLength = lengthAfter;
				}
			}
		}
	}

	if (bestTargetIDFrom != -1) {
		INFO("bestTargetIDFrom " << bestTargetIDFrom);
		INFO("bestTargetIDTo " << bestTargetIDTo);
		INFO("maximalReward " << maximalReward);
		INFO("minimalLength " << minimalLength);
		int testingRelocateID = actualVNS[bestTargetIDFrom];
		if (bestTargetIDFrom > bestTargetIDTo) {
			if (bestTargetIDFrom < actualVNSGOPPath.getNumTargets()) {
				actualVNSGOPPath.removePoint(bestTargetIDFrom);
			}
			actualVNS.erase(actualVNS.begin() + bestTargetIDFrom);

			if (bestTargetIDTo <= actualVNSGOPPath.getNumTargets()) {
				actualVNSGOPPath.addPoint(testingRelocateID, bestTargetIDTo);
			}
			actualVNS.insert(actualVNS.begin() + bestTargetIDTo, testingRelocateID);
		} else {
			if (bestTargetIDTo <= actualVNSGOPPath.getNumTargets()) {
				actualVNSGOPPath.addPoint(testingRelocateID, bestTargetIDTo);
			}
			actualVNS.insert(actualVNS.begin() + bestTargetIDTo, testingRelocateID);

			if (bestTargetIDFrom < actualVNSGOPPath.getNumTargets()) {
				actualVNSGOPPath.removePoint(bestTargetIDFrom);
			}
			actualVNS.erase(actualVNS.begin() + bestTargetIDFrom);
		}
	}
	INFO("onePointMove() end");
}

template<typename T>
void VnsPrmOPSolver<T>::optimize_path(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS) {
	std::vector<IndexSOP> returnedPath = actualVNSGOPPath.getPath();

	for (int var = 0; var < returnedPath.size(); ++var) {

		//INFO("sample " << var << " id " << samples[var][0].cluster_id << " pos " << samples[var][0].x << " " << samples[var][0].y );

		if (var > 0) {
			GraphNode<T> opt_from =
					nodesAllClusters[returnedPath[var - 1].clusterIndex][returnedPath[var - 1].nodeIndex];
			GraphNode<T> opt_to = nodesAllClusters[returnedPath[var].clusterIndex][returnedPath[var].nodeIndex];
			INFO(
					"optimize from " << var - 1 << " id " << opt_from.cluster_id << " to " << var << " id " << opt_to.cluster_id);
			optimize_motion_between_targets(opt_from, opt_to, 10);
		}
	}
	prm->calculate_added_points();
	this->update_roadmap_distances();
	actualVNSGOPPath.update();
	drawPath();
}

template<typename T>
void VnsPrmOPSolver<T>::optimize_motion_between_targets(GraphNode<T> from, GraphNode<T> to, int add_samples) {
	//INFO("optimize between " << from.cluster_id << " and " << to.cluster_id);
	double actual_len = nodesAllDistances[from.cluster_id][to.cluster_id][0][0];
	Point3D fromPoint = from.data.toPoint3D();
	Point3D toPoint = to.data.toPoint3D();
	double real_distance = fromPoint.distanceTo(toPoint);
	int num_added = 0;
	int num_generated_to_add = 0;
	double points_to_add = ceil(actual_len / 1500.0);
	//INFO_VAR(actual_len);
	double ellipse_area = 0;
	//nodesAllPairsSamplingDensity;
	points_to_add = add_samples;
	//INFO_VAR(points_to_add);
	while (num_added < points_to_add && num_generated_to_add < 2 * points_to_add) {
		ellipse_rand new_rand = random_point_in_ellipse(fromPoint, toPoint, actual_len, planning_state,
				sampling_limits);
		Point3DOriented new_point_inside = new_rand.point;
		//INFO("add point "<< new_point_inside.x << " , " << new_point_inside.y << " , "<<new_point_inside.z<< " between " << fromPoint.x << " , " << fromPoint.y << " , " << fromPoint.z<< " and " << toPoint.x << " , " << toPoint.y << " , " << toPoint.z);

		ellipse_area = new_rand.area;
		num_generated_to_add++;
		if (!std::isnan(new_point_inside.x) && !std::isnan(new_point_inside.y) && !std::isnan(new_point_inside.z)) {
			bool added = prm->add_point(new_point_inside);
			if (added) {
				num_added++;
			}
		}
	}
	if (ellipse_area != 0 && num_added > 0) {
		nodesAllPairsSamplingDensity[from.cluster_id][to.cluster_id] += num_added / ellipse_area;
	}
	//INFO("optimize between end");
}

template<typename T>
bool VnsPrmOPSolver<T>::insertRandom(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int num_changes) {
//INFO("insertRandom beg");
	const int sizeV = actualVNS.size() - 1;

	double actualLength = actualVNSGOPPath.getPathLength();
	double actualReward = actualVNSGOPPath.getReward();
	double bestTourReward = tourVNSGOPPath.getReward();

	double maximally_achieved_reward = MAX(actualReward, bestTourReward);
	double maxOverbudget = 0;
	int num_improvable = 0;
	int num_test_improve = 0;
	int lastImprovementIndex = 0;

	int numTestImprovableReward = 0;

	int num_unchanged_iters = 0;
	int stop_after_iters = MAX(100, num_changes / 5);

	VnsSopPath<T> bestOverRewardVNSGOPPath = actualVNSGOPPath;
	std::vector<int> bestOverRewardVNS = actualVNS;

	if (actualVNS.size() >= 3) {
		for (int var = 0; var < num_changes; ++var) {

			int targetIDFrom = randIntMinMax(0, actualVNS.size() - 1);
			int targetIDTo = randIntMinMax(0, actualVNSGOPPath.getNumTargets());
			//int targetIDTo = randIntMinMax(0, actualVNS.size() - 1);
			while (abs(targetIDFrom - targetIDTo) <= 1) {
				targetIDFrom = randIntMinMax(0, actualVNS.size() - 1);
				targetIDTo = randIntMinMax(0, actualVNSGOPPath.getNumTargets());

			}

			//checkLengths(actualVNSGOPPath,actualVNS, "insertRandom before change");

			int testingRelocateID = actualVNS[targetIDFrom];
			double testingRelocateReward = cluster_rewards[testingRelocateID];

			double lengthAfterRemove = actualLength;
			if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
				lengthAfterRemove = actualVNSGOPPath.tryToRemove(targetIDFrom);
			}

			double lengthRemoved = actualLength - lengthAfterRemove;
			double lengthAfterMove = 0;
			double addedReward = 0;
			int num_targets_after = actualVNSGOPPath.getNumTargets();
			if (targetIDFrom < actualVNSGOPPath.getNumTargets() && targetIDTo <= actualVNSGOPPath.getNumTargets()) {
				//both targetIDTo and targetIDFrom inside existing path - exchange points inside path

				lengthAfterMove = actualVNSGOPPath.tryToMove(targetIDFrom, targetIDTo);

				//check outcome
				check_try_operation(actualVNSGOPPath, actualVNS, targetIDFrom, targetIDTo, lengthAfterMove, false);

			} else if (targetIDTo <= actualVNSGOPPath.getNumTargets()) {
				//targetIDTo inside exisitng path - adding new point to path

				double lengthAfterAdd = actualVNSGOPPath.tryToAdd(testingRelocateID, targetIDTo);
				num_targets_after += 1;
				double lengthAdded = lengthAfterAdd - actualLength;
				addedReward = ((targetIDFrom >= actualVNSGOPPath.getNumTargets()) ? (testingRelocateReward) : (0));

				lengthAfterMove = actualLength - lengthRemoved + lengthAdded;

			} else {

				//targetIDTo out of path
				if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
					//targetIDFrom in path - removing int from path
					//INFO("boooooo below")
					lengthAfterMove = lengthAfterRemove;
					addedReward = -testingRelocateReward;
					num_targets_after -= 1;
				} else {
					//points outside of path moved
					lengthAfterMove = actualLength;
				}
			}

			double reward_after_change = actualReward + addedReward;
			if (reward_after_change >= optimize_initial_reward_ratio * initial_reward
					&& lengthAfterMove <= overbudget_optimize_ratio * budget) {
				std::vector<int> cluster_sequence = actualVNS;

				double larger_than_budget_ratio = 0;

				if (lengthAfterMove <= budget) {
					larger_than_budget_ratio = 1;
				} else {
					//*(lengthAfterMove-budget)/budget = 0 - 0.2
					larger_than_budget_ratio = -(lengthAfterMove / budget - 1.0) / (overbudget_optimize_ratio - 1.0)
							+ 1;
					//INFO("larger_than_budget_ratio "<<larger_than_budget_ratio<<" budget "<<budget<<" lengthAfterMove "<<lengthAfterMove)
				}

				if (reward_after_change > bestTourReward) {
					//INFO_CYAN("found better rewarded path with "<<(lengthAfterMove / budget)<<" budget overshoot with r "<<reward_after_change<<" best tour has "<<bestTourReward);
					larger_than_budget_ratio *= 10;
					this->found_better_reward = true;
				}

				double budget_limit_reward = reward_after_change * larger_than_budget_ratio / (num_targets_after + 1);

				int testingRelocateID = cluster_sequence[targetIDFrom];
				if (targetIDTo > targetIDFrom) {
					cluster_sequence.insert(cluster_sequence.begin() + targetIDTo, testingRelocateID);
					cluster_sequence.erase(cluster_sequence.begin() + targetIDFrom);
					//nodesAllPairsInRewardUsage[cluster_sequence[targetIDTo - 2]][cluster_sequence[targetIDTo - 1]] += budget_limit_reward;
					//nodesAllPairsInRewardUsage[cluster_sequence[targetIDTo - 1]][cluster_sequence[targetIDTo]] += budget_limit_reward;
				} else {
					//tady nekde chyba?
					cluster_sequence.erase(cluster_sequence.begin() + targetIDFrom);
					cluster_sequence.insert(cluster_sequence.begin() + targetIDTo, testingRelocateID);
					//nodesAllPairsInRewardUsage[cluster_sequence[targetIDTo - 1]][cluster_sequence[targetIDTo]] += budget_limit_reward;
					//nodesAllPairsInRewardUsage[cluster_sequence[targetIDTo]][cluster_sequence[targetIDTo + 1]] += budget_limit_reward;
				}
				nodesAllPairsInRewardUsage[startIndex][cluster_sequence[0]] += budget_limit_reward;
				for (int var = 1; var < num_targets_after; ++var) {
					nodesAllPairsInRewardUsage[cluster_sequence[var - 1]][cluster_sequence[var]] += budget_limit_reward;
				}
				nodesAllPairsInRewardUsage[cluster_sequence[num_targets_after - 1]][goalIndex] += budget_limit_reward;
				//add to this structure based on reward_after_change and lengthAfterMove
			}

			//INFO("test budget");
			if (lengthAfterMove <= budget && (addedReward > 0 || (actualLength - lengthAfterMove) > MIN_CHANGE_EPS)) {
				lastImprovementIndex = var;
				int testingRelocateID = actualVNS[targetIDFrom];

				//checkLengths(actualVNSGOPPath,actualVNS, "insertRandom before change");

				if (targetIDTo > targetIDFrom) {
					//move up
					if (targetIDTo <= actualVNSGOPPath.getNumTargets()) {
						actualVNSGOPPath.addPoint(testingRelocateID, targetIDTo);
					}
					if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
						//INFO("remove point "<<targetIDFrom);
						actualVNSGOPPath.removePoint(targetIDFrom);
					}
					actualVNS.insert(actualVNS.begin() + targetIDTo, testingRelocateID);
					actualVNS.erase(actualVNS.begin() + targetIDFrom);
				} else {
					if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
						//INFO("remove point "<<targetIDFrom);
						actualVNSGOPPath.removePoint(targetIDFrom);
					}
					if (targetIDTo <= actualVNSGOPPath.getNumTargets()) {
						actualVNSGOPPath.addPoint(testingRelocateID, targetIDTo);
					}
					actualVNS.erase(actualVNS.begin() + targetIDFrom);
					actualVNS.insert(actualVNS.begin() + targetIDTo, testingRelocateID);
				}
				num_unchanged_iters = 0;
				//DEBUG TESTING
				actualLength = actualVNSGOPPath.getPathLength();
				double newReward = actualVNSGOPPath.getReward();

				actualReward = newReward;
				if (actualReward > maximally_achieved_reward) {
					maximally_achieved_reward = actualReward;
				}

				if (actualLength - lengthAfterMove > 0.25) {
					INFO("wrongly calculated insertRandom");
					INFO_VAR(actualLength);
					INFO_VAR(lengthAfterMove);
					INFO_VAR(targetIDFrom);
					INFO_VAR(targetIDTo);
					INFO_VAR(lengthRemoved);
					INFO_VAR(actualVNSGOPPath.getNumTargets());
					INFO_VAR(actualVNSGOPPath.getPathLengthCalculate());
					INFO_VAR(actualVNSGOPPath.getPathLengthRear());
					actualVNSGOPPath.listIds();
					exit(1);
				}

			} else {
				num_unchanged_iters++;
			}
		}
	}
	return true;
}

template<typename T>
bool VnsPrmOPSolver<T>::exchangeRandom(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int num_changes) {
//INFO("exchangeRandom beg");
	const int sizeV = actualVNS.size() - 1;
	double actualLength = actualVNSGOPPath.getPathLength();
	double actualReward = actualVNSGOPPath.getReward();
	double bestTourReward = tourVNSGOPPath.getReward();

	double maximally_achieved_reward = MAX(actualReward, bestTourReward);

	int num_unchanged_iters = 0;
	int stop_after_iters = MAX(100, num_changes / 5);

//checkLengths(actualVNSGOPPath,actualVNS, "exchangeRandom beign");

	int lastImprovementIndex = 0;
	int numTestImprovableReward = 0;

	VnsSopPath<T> bestOverRewardVNSGOPPath = actualVNSGOPPath;
	std::vector<int> bestOverRewardVNS = actualVNS;

	if (actualVNS.size() >= 4 && actualVNSGOPPath.getNumTargets() > 0) {
		for (int var = 0; var < num_changes; ++var) {
			/*
			 if ( num_unchanged_iters > stop_after_iters ) {
			 INFO_CYAN("stopping exchangeRandom after unsuccessful "<<num_unchanged_iters<<" iterations");
			 break;
			 }
			 */

			double minLength = actualLength;
			double maxAddReward = 0;

			int targetIDFrom = randIntMinMax(0, actualVNS.size() - 1);
			int targetIDTo = randIntMinMax(0, actualVNSGOPPath.getNumTargets() - 1);
			//int targetIDTo = randIntMinMax(0, actualVNS.size() - 1);
			while (abs(targetIDFrom - targetIDTo) <= 2) {
				targetIDFrom = randIntMinMax(0, actualVNS.size() - 1);
				targetIDTo = randIntMinMax(0, actualVNSGOPPath.getNumTargets() - 1);
				//targetIDTo = randIntMinMax(0, actualVNS.size() - 1);
			}

			//checkLengths(actualVNSGOPPath,actualVNS, "exchangeRandom before change");

			int testingRelocateFromID = actualVNS[targetIDFrom];
			double testingRelocateFromReward = cluster_rewards[testingRelocateFromID];

			if (abs(targetIDFrom - targetIDTo) > 2) {
				int testingRelocateToID = actualVNS[targetIDTo];
				double testingRelocateToReward = cluster_rewards[testingRelocateToID];
				double addedReward = 0;
				double lengthAfterMove = 0;
				int num_targets_after = actualVNSGOPPath.getNumTargets();
				if (targetIDFrom < actualVNSGOPPath.getNumTargets() && targetIDTo < actualVNSGOPPath.getNumTargets()) {
					//exchange two nodes that are in the feasible path
					//INFO("if before tryToExchangeLowerBound");

					lengthAfterMove = actualVNSGOPPath.tryToExchange(targetIDFrom, targetIDTo);

					check_try_operation(actualVNSGOPPath, actualVNS, targetIDFrom, targetIDTo, lengthAfterMove, true);

				} else {
					//testingRelocateFrom is out of feasible

					double lengthAfterReplaceTo = actualVNSGOPPath.tryToReplace(testingRelocateFromID, targetIDTo);
					addedReward += testingRelocateFromReward - testingRelocateToReward;

					lengthAfterMove = lengthAfterReplaceTo;
				}

				//////

				double reward_after_change = actualReward + addedReward;
				if (reward_after_change >= optimize_initial_reward_ratio * initial_reward
						&& lengthAfterMove <= overbudget_optimize_ratio * budget) {
					double larger_than_budget_ratio = 0;
					if (lengthAfterMove <= budget) {
						larger_than_budget_ratio = 1;
					} else {
						larger_than_budget_ratio = -(lengthAfterMove / budget - 1.0) / (overbudget_optimize_ratio - 1.0)
								+ 1;
						//larger_than_budget_ratio = (lengthAfterMove / budget - 1.0) / (overbudget_optimize_ratio - 1.0);
						//INFO("larger_than_budget_ratio "<<larger_than_budget_ratio<<" budget "<<budget<<" lengthAfterMove "<<lengthAfterMove)
					}

					if (reward_after_change > bestTourReward) {
						//INFO_CYAN("found better rewarded path with "<<(lengthAfterMove / budget)<<" budget overshoot with r "<<reward_after_change<<" best tour has "<<bestTourReward);
						larger_than_budget_ratio *= 10;
						this->found_better_reward = true;
					}

					double budget_limit_reward = reward_after_change * larger_than_budget_ratio
							/ (num_targets_after + 1);

					std::vector<int> cluster_sequence = actualVNS;
					int temp_move = cluster_sequence[targetIDTo];
					cluster_sequence[targetIDTo] = cluster_sequence[targetIDFrom];
					cluster_sequence[targetIDFrom] = temp_move;

					//nodesAllPairsInRewardUsage[cluster_sequence[targetIDTo - 1]][cluster_sequence[targetIDTo]] += budget_limit_reward;
					//nodesAllPairsInRewardUsage[cluster_sequence[targetIDTo]][cluster_sequence[targetIDTo + 1]] += budget_limit_reward;
					//nodesAllPairsInRewardUsage[cluster_sequence[targetIDFrom - 1]][cluster_sequence[targetIDFrom]] += budget_limit_reward;
					//nodesAllPairsInRewardUsage[cluster_sequence[targetIDFrom]][cluster_sequence[targetIDFrom + 1]] += budget_limit_reward;
					nodesAllPairsInRewardUsage[startIndex][cluster_sequence[0]] += budget_limit_reward;
					for (int var = 1; var < num_targets_after; ++var) {
						nodesAllPairsInRewardUsage[cluster_sequence[var - 1]][cluster_sequence[var]] +=
								budget_limit_reward;
					}
					nodesAllPairsInRewardUsage[cluster_sequence[num_targets_after - 1]][goalIndex] +=
							budget_limit_reward;
					//nodesAllPairsInRewardUsage //add to this structure based on reward_after_change and lengthAfterMove
				}
				//////

				if (lengthAfterMove <= budget
						&& (addedReward > maxAddReward || (minLength - lengthAfterMove) > MIN_CHANGE_EPS)) {
					lastImprovementIndex = var;
					int testingRelocateFromID = actualVNS[targetIDFrom];
					int testingRelocateToID = actualVNS[targetIDTo];
					//INFO("before change");
					if (targetIDTo > targetIDFrom) {
						actualVNSGOPPath.removePoint(targetIDTo);
						actualVNSGOPPath.addPoint(testingRelocateFromID, targetIDTo);
						if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
							actualVNSGOPPath.removePoint(targetIDFrom);
							actualVNSGOPPath.addPoint(testingRelocateToID, targetIDFrom);
						}
						int temp = actualVNS[targetIDTo];
						actualVNS[targetIDTo] = actualVNS[targetIDFrom];
						actualVNS[targetIDFrom] = temp;
					} else {
						if (targetIDFrom < actualVNSGOPPath.getNumTargets()) {
							actualVNSGOPPath.removePoint(targetIDFrom);
							actualVNSGOPPath.addPoint(testingRelocateToID, targetIDFrom);
						}
						actualVNSGOPPath.removePoint(targetIDTo);
						actualVNSGOPPath.addPoint(testingRelocateFromID, targetIDTo);

						int temp = actualVNS[targetIDTo];
						actualVNS[targetIDTo] = actualVNS[targetIDFrom];
						actualVNS[targetIDFrom] = temp;
					}
					num_unchanged_iters = 0;
					//actualVNSGOPPath.evaluateUsage();
					double newReward = actualVNSGOPPath.getReward();

					actualLength = actualVNSGOPPath.getPathLength();
					actualReward = newReward;
					if (actualReward > maximally_achieved_reward) {
						maximally_achieved_reward = actualReward;
					}

					if (actualLength - lengthAfterMove > 0.02) {
						INFO("wrongly calculated exchangeRandom");
						INFO_VAR(actualLength);
						INFO_VAR(lengthAfterMove);
						INFO_VAR(targetIDFrom);
						INFO_VAR(targetIDTo);
						INFO_VAR(actualVNSGOPPath.getNumTargets());
						INFO_VAR(actualVNSGOPPath.getPathLengthCalculate());
						INFO_VAR(actualVNSGOPPath.getPathLengthRear());
						actualVNSGOPPath.listIds();
						exit(1);
					}
					//INFO("IMPROVED NORMAL");
				} else {
					num_unchanged_iters++;
				}
			}
		}
	}
	return true;
}

template<typename T>
bool VnsPrmOPSolver<T>::check_try_operation(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS,
		int targetIDFrom, int targetIDTo, double lengthAfterMove, bool relocate) {

	if ( DEBUG_DOP_TRY_OPERATIONS && !relocate) {
		INFO_VAR(DEBUG_DOP_TRY_OPERATIONS);
		int testingRelocateID = actualVNS[targetIDFrom];

		VnsSopPath<T> copyAdd = actualVNSGOPPath;
		if (targetIDTo > targetIDFrom) {
			copyAdd.addPoint(testingRelocateID, targetIDTo);
			if (targetIDFrom < copyAdd.getNumTargets()) {
				copyAdd.removePoint(targetIDFrom);
			}
		} else {
			if (targetIDFrom < copyAdd.getNumTargets()) {
				copyAdd.removePoint(targetIDFrom);
			}
			copyAdd.addPoint(testingRelocateID, targetIDTo);
		}
		copyAdd.update();
		double lengthAfterAddCopy = copyAdd.getPathLength();
		if (fabs(lengthAfterAddCopy - lengthAfterMove) > 0.1) {
			ERROR("lengthAfterMove does not match lengthAfterAdd");
			ERROR("insertRandom from " << targetIDFrom << " to " << targetIDTo);
			ERROR("id " << testingRelocateID);
			ERROR(lengthAfterAddCopy << " and " << lengthAfterMove);
			actualVNSGOPPath.listIds();
			exit(1);
		}
	}

	if ( DEBUG_DOP_TRY_OPERATIONS && relocate) {
		INFO_VAR(DEBUG_DOP_TRY_OPERATIONS);
		VnsSopPath<T> copyAdd = actualVNSGOPPath;
		int testingRelocateFromCopyID = actualVNS[targetIDFrom];
		int testingRelocateToCopyID = actualVNS[targetIDTo];

		if (targetIDTo > targetIDFrom) {
			copyAdd.removePoint(targetIDTo);
			copyAdd.addPoint(testingRelocateFromCopyID, targetIDTo);
			if (targetIDFrom < copyAdd.getNumTargets()) {
				copyAdd.removePoint(targetIDFrom);
				copyAdd.addPoint(testingRelocateToCopyID, targetIDFrom);
			}
		} else {
			if (targetIDFrom < copyAdd.getNumTargets()) {
				copyAdd.removePoint(targetIDFrom);
				copyAdd.addPoint(testingRelocateToCopyID, targetIDFrom);
			}
			copyAdd.removePoint(targetIDTo);
			copyAdd.addPoint(testingRelocateFromCopyID, targetIDTo);
		}
		copyAdd.update();
		double lengthAfterAddCopy = copyAdd.getPathLength();
		if (fabs(lengthAfterAddCopy - lengthAfterMove) > 0.1) {
			ERROR("lengthAfterMove does not match lengthAfterAdd");
			ERROR("exchangeRandom from " << targetIDFrom << " to " << targetIDTo);
			ERROR(lengthAfterAddCopy << " and " << lengthAfterMove);
			//actualVNSGOPPath.listIds();
			exit(1);
		}
	}
	return true;
}

template<typename T>
void VnsPrmOPSolver<T>::pathInsert(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int numExchanges) {
//INFO("pathInsert begin");
	const int sizeV = actualVNS.size() - 1;
	if (sizeV >= 2) {
		for (int var = 0; var < numExchanges; ++var) {
			std::unordered_set<int> randNumbersSet;
			while (randNumbersSet.size() < 3) {
				randNumbersSet.insert(randIntMinMax(0, sizeV));
			}
			std::vector<int> randNumbers;
			int i = 0;
			for (auto it = randNumbersSet.begin(); it != randNumbersSet.end(); ++it) {
				randNumbers.push_back(*it);
			}
			CSort<int>::quicksort(&randNumbers, 0, randNumbers.size() - 1);

			int exchangeFromStart = randNumbers[0];
			int exchangeFromEnd = randNumbers[1];
			int insertTo = randNumbers[2];
			bool insertUp = true;
			if (randIntMinMax(0, 1) == 1) {
				insertTo = randNumbers[0];
				exchangeFromStart = randNumbers[1];
				exchangeFromEnd = randNumbers[2];
				insertUp = false;
			}

			if (insertUp) {
				//insert up

				std::vector<int> newVec(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);

				actualVNS.insert(actualVNS.begin() + insertTo, newVec.begin(), newVec.end());
				actualVNS.erase(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);

				if (insertTo < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.addPoint(newVec, insertTo);
				}
				if (exchangeFromStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeFromStart,
							MIN(exchangeFromEnd, actualVNSGOPPath.getNumTargets() - 1));
				}

			} else {
				//insert bellow

				std::vector<int> newVec(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);

				actualVNS.erase(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);
				actualVNS.insert(actualVNS.begin() + insertTo, newVec.begin(), newVec.end());

				if (exchangeFromStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeFromStart,
							MIN(exchangeFromEnd, actualVNSGOPPath.getNumTargets() - 1));
				}
				if (insertTo < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.addPoint(newVec, insertTo);
				}

			}

			fitDOPtoBudget(actualVNSGOPPath, actualVNS);

			//actualVNSGOPPath.evaluateUsage();
		}
	}
}

template<typename T>
void VnsPrmOPSolver<T>::pathExchange(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS, int numExchanges) {
//INFO("pathExchange begin");
//INFO("pathExchange "<<numExchanges<<" begin");
	const int sizeV = actualVNS.size() - 1;
	if (sizeV >= 3) {
		for (int var = 0; var < numExchanges; ++var) {
			std::unordered_set<int> randNumbersSet;
			while (randNumbersSet.size() < 4) {
				randNumbersSet.insert(randIntMinMax(0, sizeV));
			}
			std::vector<int> randNumbers;
			for (auto it = randNumbersSet.begin(); it != randNumbersSet.end(); ++it) {
				randNumbers.push_back(*it);
			}
			CSort<int>::quicksort(&randNumbers, 0, randNumbers.size() - 1);

			int exchangeFromStart = randNumbers[0];
			int exchangeFromEnd = randNumbers[1];
			int exchangeToStart = randNumbers[2];
			int exchangeToEnd = randNumbers[3];
			bool exchangeUp = true;
			if (randIntMinMax(0, 1) == 1) {
				exchangeFromStart = randNumbers[2];
				exchangeFromEnd = randNumbers[3];
				exchangeToStart = randNumbers[0];
				exchangeToEnd = randNumbers[1];
				exchangeUp = false;
			}

			if (exchangeUp) {
				std::vector<int> newVecFrom(actualVNS.begin() + exchangeFromStart,
						actualVNS.begin() + exchangeFromEnd + 1);	//is below
				std::vector<int> newVecTo(actualVNS.begin() + exchangeToStart, actualVNS.begin() + exchangeToEnd + 1);//is above

				actualVNS.erase(actualVNS.begin() + exchangeToStart, actualVNS.begin() + exchangeToEnd + 1);
				actualVNS.insert(actualVNS.begin() + exchangeToStart, newVecFrom.begin(), newVecFrom.end());
				actualVNS.erase(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);
				actualVNS.insert(actualVNS.begin() + exchangeFromStart, newVecTo.begin(), newVecTo.end());

				//INFO("add newVecFrom");
				if (exchangeToStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeToStart,
							MIN(exchangeToEnd, actualVNSGOPPath.getNumTargets() - 1));
					actualVNSGOPPath.addPoint(newVecFrom, exchangeToStart);
				}
				//INFO("add newVecTo");
				if (exchangeFromStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeFromStart,
							MIN(exchangeFromEnd, actualVNSGOPPath.getNumTargets() - 1));
					actualVNSGOPPath.addPoint(newVecTo, exchangeFromStart);
				}

			} else {
				std::vector<int> newVecFrom(actualVNS.begin() + exchangeFromStart,
						actualVNS.begin() + exchangeFromEnd + 1);
				std::vector<int> newVecTo(actualVNS.begin() + exchangeToStart, actualVNS.begin() + exchangeToEnd + 1);

				actualVNS.erase(actualVNS.begin() + exchangeFromStart, actualVNS.begin() + exchangeFromEnd + 1);
				actualVNS.insert(actualVNS.begin() + exchangeFromStart, newVecTo.begin(), newVecTo.end());

				actualVNS.erase(actualVNS.begin() + exchangeToStart, actualVNS.begin() + exchangeToEnd + 1);
				actualVNS.insert(actualVNS.begin() + exchangeToStart, newVecFrom.begin(), newVecFrom.end());

				//INFO("size after "<<actualVNS.size());

				//INFO("add newVecTo");
				if (exchangeFromStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeFromStart,
							MIN(exchangeFromEnd, actualVNSGOPPath.getNumTargets() - 1));
					actualVNSGOPPath.addPoint(newVecTo, exchangeFromStart);
				}

				//INFO("add newVecFrom");
				if (exchangeToStart < actualVNSGOPPath.getNumTargets()) {
					actualVNSGOPPath.removePoint(exchangeToStart,
							MIN(exchangeToEnd, actualVNSGOPPath.getNumTargets() - 1));
					actualVNSGOPPath.addPoint(newVecFrom, exchangeToStart);
				}

			}

			fitDOPtoBudget(actualVNSGOPPath, actualVNS);

			//actualVNSGOPPath.evaluateUsage();
		}
	}
}

template<typename T>
void VnsPrmOPSolver<T>::drawPath(int usleepTime, VnsSopPath<T> * toShow) {
	if (canvas) {

		std::vector<IndexSOP> returnedPath;
		if (toShow != NULL) {
			returnedPath = toShow->getPath();
		} else {
			returnedPath = this->tourVNSGOPPath.getPath();
		}

		if (returnedPath.size() >= 2) {
			std::vector<HeapNode<T>> plan_all;
			INFO("drawPath")
			for (int var = 1; var < returnedPath.size(); ++var) {
				int node_id_from =
						nodesAllClusters[returnedPath[var - 1].clusterIndex][returnedPath[var - 1].nodeIndex].id;
				int node_id_to = nodesAllClusters[returnedPath[var].clusterIndex][returnedPath[var].nodeIndex].id;
				plan_with_length<T> found_path = prm->plan(node_id_from, node_id_to);
				if (var == 1) {
					plan_all.insert(std::end(plan_all), std::begin(found_path.plan), std::end(found_path.plan));
				} else {
					plan_all.insert(std::end(plan_all), std::begin(found_path.plan) + 1, std::end(found_path.plan));
				}

			}
			drawVisibilityPath(usleepTime, &plan_all);
		}

		CShape blackPoint("black", "black", 2, 5);

		*canvas << canvas::CLEAR << "pathpoint" << "pathpoint" << canvas::POINT;
		for (int var = 0; var < returnedPath.size(); ++var) {
			GraphNode<T> gn = nodesAllClusters[returnedPath[var].clusterIndex][returnedPath[var].nodeIndex];
			Coords coord(gn.data.x, gn.data.y);
			*canvas << blackPoint << coord;
		}
		*canvas << canvas::END;

		canvas->redraw();

	}
}

template<typename T>
void VnsPrmOPSolver<T>::checkConsistency(VnsSopPath<T> &actualVNSGOPPath, std::vector<int> &actualVNS) {
	if ( DEBUG_CHECK_CONSISTENCY) {
		INFO("checkConsistency begin");
		if (actualVNSGOPPath.getPathLength() > budget) {
			ERROR(
					"inconsistent actualVNSGOPPath is over budget " << actualVNSGOPPath.getPathLength() << " > " << budget);
			exit(1);
		}

		for (int var1 = 0; var1 < actualVNS.size(); ++var1) {
			if (var1 < actualVNSGOPPath.getNumTargets()) {
				if (actualVNS[var1] != actualVNSGOPPath.getTarget(var1)) {
					ERROR(
							"inconsistent actualVNS not same as actualVNSGOPPath at position" << var1 << " ids are " << actualVNS[var1] << " and " << actualVNSGOPPath.getTarget(var1));
					exit(1);
				}
			}
			for (int var2 = var1 + 1; var2 < actualVNS.size(); ++var2) {
				if (actualVNS[var1] == actualVNS[var2]) {
					ERROR("inconsistent actualVNS repeat " << var1 << " and " << var2);
					for (int nodeID = 0; nodeID < actualVNS.size(); ++nodeID) {
						INFO(nodeID << " id " << actualVNS[nodeID]);
					}
					exit(1);
				}
			}
		}
		INFO("checkConsistency end");
	}
}

template<typename T>
void VnsPrmOPSolver<T>::defineResultLog(void) {
	static bool resultLogInitialized = false;
	if (!resultLogInitialized) {
		resultLog << result::newcol << "NAME";
		resultLog << result::newcol << "METHOD";
		resultLog << result::newcol << "CTIME";
		resultLog << result::newcol << "NUM_ITERS";
		resultLog << result::newcol << "BUDGET";
		resultLog << result::newcol << "REWARDS";
		resultLog << result::newcol << "MAX_ACHIEVABLE_REWARDS";
		resultLog << result::newcol << "LENGTH";
		resultLog << result::newcol << "NUM_ITERS_LAST_IMPR";
		resultLog << result::newcol << "CTIME_LAST_IMPR";
		resultLog << result::newcol << "MAX_ALLOWED_CALC_TIME_MS";
		resultLog << result::newcol << "GREEDY_INITIAL_SOLUTION";
		resultLog << result::newcol << "RESULT_TARGET_IDS";
		resultLog << result::newcol << "RESULT_CLUSTER_IDS";
		resultLog << result::newcol << "SOLUTION_TIME_REWARD_LENGTH_IMPROVEMENTS";
		resultLog << result::newcol << "LOWER_BOUND_ABOVE_BUDGET_THEN_SKIP";
		resultLog << result::newcol << "PLANNING_STATE_TYPE";
		resultLog << result::newcol << "INITIAL_PRM_SAMPLES";
		resultLog << result::newcol << "EXTENSION_PRM_SAMPLES";
		resultLog << result::newcol << "DUBINS_RADIUS";
		resultLog << result::newcol << "OPTIMIZE_INITIAL_REWARD_RATIO";
		resultLog << result::newcol << "OVERBUDGET_OPTIMIZE_RATIO";
		resultLogInitialized = true;
	}
}

template<class T>
void VnsPrmOPSolver<T>::set_borders(std::vector<HeapNode<HeapPoint3D> *> borders) {
	sampling_limits.rand_max_x = -std::numeric_limits<double>::max();
	sampling_limits.rand_max_y = sampling_limits.rand_max_x;
	sampling_limits.rand_max_z = sampling_limits.rand_max_x;
	sampling_limits.rand_min_x = std::numeric_limits<double>::max();
	sampling_limits.rand_min_y = sampling_limits.rand_min_x;
	sampling_limits.rand_min_z = sampling_limits.rand_min_x;
	for (int var = 0; var < borders.size(); ++var) {
		sampling_limits.rand_max_x = MAX(borders[var]->data.x, sampling_limits.rand_max_x);
		sampling_limits.rand_max_y = MAX(borders[var]->data.y, sampling_limits.rand_max_y);
		sampling_limits.rand_max_z = MAX(borders[var]->data.z, sampling_limits.rand_max_z);
		sampling_limits.rand_min_x = MIN(borders[var]->data.x, sampling_limits.rand_min_x);
		sampling_limits.rand_min_y = MIN(borders[var]->data.y, sampling_limits.rand_min_y);
		sampling_limits.rand_min_z = MIN(borders[var]->data.z, sampling_limits.rand_min_z);
	}
}

template<typename T>
void VnsPrmOPSolver<T>::fillResultRecord(int numIters, double length, int numItersLastImprovement,
		long timeLastImprovement, std::vector<ImprovementLogRecord> improvementLog) {
	long t[3] = { 0, 0, 0 };
	this->tSolve.addTime(t);
	std::stringstream tourNodes;
	std::stringstream tourClusters;
	std::stringstream tourRewards;
	double final_reward, final_length;

	final_reward = this->finalTourVNSGOPPath.getReward();
	final_length = this->finalTourVNSGOPPath.getPathLength();
	std::vector<std::vector<GraphNode<T>> > samples = this->finalTourVNSGOPPath.getSamples();
	GraphNode<T> dummy = GraphNode<T>();
	std::vector<GraphNode<T>> solution = this->finalTourVNSGOPPath.getPathNeighAngIds(dummy);
	for (int var = 0; var < solution.size(); ++var) {
		if (var != 0) {
			tourNodes << ",";
			tourClusters << ",";
			tourRewards << ",";
		}
		tourNodes << solution[var].id;
		tourClusters << solution[var].cluster_id;
		tourRewards << solution[var].reward;
	}

	INFO("fillResultRecord:")
	INFO("tour cluster IDs:   " << tourClusters.str());
	INFO("tour node IDs:      " << tourNodes.str());
	INFO("tour cluster rews:  " << tourRewards.str());

	std::stringstream tourImprovements;
	int S = improvementLog.size();
	for (int var = 0; var < S; ++var) {
		if (var != 0) {
			tourImprovements << ",";
		}
		tourImprovements << improvementLog[var].timeMS << "|" << improvementLog[var].reward << "|"
				<< improvementLog[var].length << "|" << improvementLog[var].iter;
	}

	resultLog << result::newrec << name << getMethod() << t[0] << numIters << this->budget << final_reward
			<< maximalRewardAll << final_length << numItersLastImprovement << timeLastImprovement
			<< maximal_calculation_time_MS << set_greedy_initial_solution << tourNodes.str() << tourClusters.str()
			<< tourImprovements.str() << lower_bound_above_budget_then_skip << planning_state_type_str
			<< initial_prm_size << extension_prm_size << dubins_radius << optimize_initial_reward_ratio
			<< overbudget_optimize_ratio;

}

template<typename T>
void VnsPrmOPSolver<T>::save(void) {
	INFO("saving");
	std::string dir;
//updateResultRecordTimes(); //update timers as load and initilization is outside class
//DEBUG("LOAD_TIME_CPU: " << tLoad.cpuTime());
//DEBUG("INIT_TIME_CPU: " << tInit.cpuTime());
//DEBUG("SAVE_TIME_CPU: " << tSave.cpuTime());
	INFO_RED(" saving dir "<<dir)
	INFO_RED(" output dir "<<output)
	DEBUG("SOLVE_TIME_CPU: " << tSolve.cpuTime());
	INFO("finalPath")
	std::vector<IndexSOP> finalPath;
	finalPath = finalTourVNSGOPPath.getPath();

	if (SAVE_SETTINGS) {
		INFO_RED(" saving dir0.5 "<<dir)
		saveSettings(getOutputIterPath(config.get<std::string>("settings"), dir));
		INFO_RED(" saving dir1 "<<dir)
	}
	if (SAVE_INFO) {
		INFO_RED(" saving dir1.5 "<<dir)
		saveInfo(getOutputIterPath(config.get<std::string>("info"), dir));
		INFO_RED(" saving dir2 "<<dir)
	}
	if (SAVE_RESULTS) {
		INFO("SAVE_RESULTS")
		INFO_RED(" saving dir3 "<<dir)
		std::string file = getOutputIterPath(config.get<std::string>("result-path"), dir);
		assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		//std::ofstream ofs(file.c_str());
		//assert_io(ofs.good(), "Cannot create path '" + file + "'");
		//ofs << std::setprecision(14);
		VnsSopPath<T> *toShow = NULL;
		save_prm_point_path(config.get<std::string>("result-path"), toShow);
	}
	INFO_RED(" saving dir "<<dir)
	INFO("save gop.txt")
	std::string file2 = getOutputIterPath("gop.txt", dir);
	assert_io(createDirectory(dir), "Can not create file in path'" + file2 + "'");
	std::ofstream ofs2(file2.c_str());
	assert_io(ofs2.good(), "Cannot create path '" + file2 + "'");
	ofs2 << std::setprecision(14);

	INFO("save finalTourVNSGOPPath")
	int fps = finalPath.size();
	for (int var = 0; var < fps; ++var) {
		ofs2 << finalPath[var].clusterIndex << " " << finalPath[var].nodeIndex << std::endl;
	}

	assert_io(ofs2.good(), "Error occur during path saving");
	ofs2.close();

	if (SAVE_TARGETS) {
		INFO("SAVE_TARGETS")
		std::string file = getOutputIterPath(config.get<std::string>("targets"), dir);
		assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream ofs(file.c_str());
		assert_io(ofs.good(), "Cannot create path '" + file + "'");
		ofs << std::setprecision(14);
		for (int var = 0; var < num_clusters; ++var) {
			for (int var2 = 0; var2 < nodesAllClusters[var].size(); ++var2) {
				ofs << nodesAllClusters[var][var2].id << DD << nodesAllClusters[var][var2].data.x << DD
						<< nodesAllClusters[var][var2].data.y << DD << nodesAllClusters[var][var2].reward << DD
						<< nodesAllClusters[var][var2].cluster_id << std::endl;
			}
		}
		assert_io(ofs.good(), "Error occur during path saving");
		ofs.close();
	}

	//save prm points

	std::stringstream prm_points_filename;
	prm_points_filename << "final_prm_points.txt";
	save_prm_points(prm_points_filename.str());
	std::stringstream prm_points_path_filename;
	prm_points_path_filename << "final_prm_point_path.txt";
	VnsSopPath<T> *toShow = NULL;
	save_prm_point_path(prm_points_path_filename.str(), toShow);
	if (canvas) { // map must be set
		*canvas << canvas::CLEAR << "ring";
		if (config.get<bool>("draw-path")) {
			drawPath();
		}
		saveCanvas();
	}

	if (sample_by_time) {
		std::string file3 = getOutputIterPath("sampled_trajectory.path", dir);
		assert_io(createDirectory(dir), "Can not create file in path'" + file3 + "'");
		std::ofstream ofs3(file3.c_str());
		assert_io(ofs3.good(), "Cannot create path '" + file3 + "'");
		ofs3 << std::setprecision(14);
		double speed_time_sample_dist = fly_speed * sample_time_s;
		std::vector<T> samples_traj = finalTourVNSGOPPath.getPathSampled(speed_time_sample_dist);
		std::vector<std::vector<double>> trajectory_vectors = getTrajectoryPointVectors(samples_traj);
		int var2_max = trajectory_vectors[0].size();
		for (int var = 0; var < trajectory_vectors.size(); ++var) {
			for (int var2 = 0; var2 < var2_max; ++var2) {
				ofs3 << trajectory_vectors[var][var2];
				if (var2 != var2_max - 1) {
					ofs3 << " ";
				}
			}
			ofs3 << std::endl;
		}
		assert_io(ofs3.good(), "Error occur during path saving");
		ofs3.close();
	}

}

template<typename T>
void VnsPrmOPSolver<T>::createOPproblem() {
	//std::string filename = "op_problem.op";
	std::stringstream filenamess;
	filenamess << name << "_b_" << budget << "_problem.op";
	std::string filename = filenamess.str();
	std::string dir = output;

	std::string file = getOutputIterPath(filename, dir);
	assert_io(createDirectory(getDirname(file)), "Can not create file in path'" + file + "'");
	int dimension = 0;
	for (int var = 0; var < nodesAllClusters.size(); ++var) {
		for (int var2 = 0; var2 < nodesAllClusters[var].size(); ++var2) {
			dimension += 1;
		}
	}

	std::ofstream ofs(file.c_str());
	crl::assert_io(ofs.good(), "Cannot create path '" + file + "'");

	//nodesAllDistances
	ofs << "NAME: " << this->name << std::endl;
	ofs << "TYPE: TSP" << std::endl;
	ofs << "COMMENT: by Robert start is first set, goal is second" << std::endl;
	ofs << "DIMENSION: " << dimension << std::endl;
	ofs << "TMAX: " << this->budget << std::endl;
	ofs << "START_SET: 0" << std::endl;
	ofs << "END_SET: 1" << std::endl;
	ofs << "SETS: " << nodesAllClusters.size() << std::endl;
	ofs << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl;
	ofs << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl;
	ofs << "EDGE_WEIGHT_SECTION" << std::endl;
	int counter = 0;
	for (int clfrom = 0; clfrom < nodesAllClusters.size(); ++clfrom) {
		for (int nodefrom = 0; nodefrom < nodesAllClusters[clfrom].size(); ++nodefrom) {
			for (int clto = 0; clto < nodesAllClusters.size(); ++clto) {
				for (int nodeto = 0; nodeto < nodesAllClusters[clto].size(); ++nodeto) {
					ofs << " " << nodesAllDistances[clfrom][clto][nodefrom][nodeto];
					counter++;
					if (counter == 16) {
						ofs << std::endl;
						counter = 0;
					}
				}
			}
			ofs << std::endl;
			counter = 0;
		}
	}

	ofs << "GTSP_SET_SECTION: set_id set_profit id-vertex-list" << std::endl;
	for (int clfrom = 0; clfrom < nodesAllClusters.size(); ++clfrom) {
		ofs << nodesAllClusters[clfrom][0].cluster_id << " " << nodesAllClusters[clfrom][0].reward << " ";
		for (int nodefrom = 0; nodefrom < nodesAllClusters[clfrom].size(); ++nodefrom) {
			ofs << (nodesAllClusters[clfrom][nodefrom].id + 1);
		}
		ofs << std::endl;
	}
	crl::assert_io(ofs.good(), "Error occur during path saving");
	ofs.close();
}

template<typename T>
void VnsPrmOPSolver<T>::fillImprovementRecord(ImprovementLogRecord& initialImprovementRecord, long timeLastImprovement,
		int iter) {
	initialImprovementRecord.length = tourVNSGOPPath.getPathLength();
	initialImprovementRecord.reward = tourVNSGOPPath.getReward();
	initialImprovementRecord.timeMS = timeLastImprovement;
	initialImprovementRecord.iter = iter;
}

}
#endif /* SRC_VNSGOP_VNSGOP_H_ */
