//============================================================================
// Name        : gop.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <unistd.h>
#include "iostream"
#include <string>

#include <log4cxx/appender.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/layout.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/simplelayout.h>
#include <log4cxx/level.h>

#include <boost/foreach.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include "crl/logging.h"
#include "crl/exceptions.h"
#include "crl/timerN.h"
#include "crl/perf_timer.h"
#include "crl/gui/shapes.h"
#include "crl/gui/guifactory.h"
#include "crl/gui/win_adjust_size.h"

#include "canvasview_coords.h"

#include <crl/config.h>
#include "crl/boost_args_config.h"

#include "vns_obst_op_solver.h"
#include "obst_op_loader.h"

const std::string GOP_VERSION = "1.0";

using crl::logger;

namespace po = boost::program_options;
namespace fs = boost::filesystem;
#define LOGGER_NAME "lp_gop"

using namespace crl;
using namespace crl::gui;

typedef crl::gui::CCanvasBase Canvas;

typedef std::vector<Coords> CoordsVector;
typedef std::vector<CoordsVector> CoordsVectorVector;

crl::gui::CGui *g = 0;

/// ----------------------------------------------------------------------------
/// Program options variables
/// ----------------------------------------------------------------------------
std::string guiType = "none";
std::string problemFile;

crl::CConfig guiConfig;
crl::CConfig gopConfig;
std::string canvasOutput = "";
bool is_displayable = true;

#define GUI(x)  if(gui) { x;}

crl::CConfig & addCommonConfig(crl::CConfig & config) {
	std::cout << "addCommonConfig" << std::endl;
	config.add<std::string>("output", "output directory to store particular results and outputs", "./results/");
	config.add<std::string>("results", "result log file, it will be placed in output directory", "results.log");
	config.add<std::string>("info", "information file, it will be placed in particular experiment directory",
			"info.txt");
	config.add<std::string>("settings", "store configurations in boost::program_options config file format ",
			"settings.txt");
	config.add<std::string>("result-path", "file name for the final found path (ring) as sequence of points",
			"path.txt");
	config.add<std::string>("result-canvas-output",
			"file name for final canvas output (eps,png,pdf,svg) are supported");
	config.add<std::string>("result-canvas-suffixes",
			"comman separated list of exptensis e.g. png,pdf.  if specified  several result images are created, with particular suffix to the resultCanvasOutput");
	config.add<std::string>("name",
			"name used in result log as user identification if not set a default values (cities) is used");
	config.add<int>("iteration", "set particular interation, otherwise all interations (batch) are performed", -1);
	config.add<int>("batch", "number of iterations from 0 to batch (not included) ", -1);
	config.add<bool>("continue",
			"in batch mode, partial results are loaded and checked, only missing iterations are computed ", false);
	config.add<bool>("save-results", "disable/enable save results,configs and so on", true);
	config.add<bool>("save-info", "disable/enable save info", true);
	config.add<bool>("save-settings", "disable/enable save settings", true);

	config.add<bool>("save-visual", "disable/enable save visualization results, canvas be", true);
	config.add<bool>("verbose-result-log", "disable/enable printing results log into logger", false);
// end basic config

	config.add<bool>("draw-stations", "Enable/Disable drawing stations", true);
	config.add<bool>("draw-ring", "Enable/Disable drawing ring in the final shoot", true);
	config.add<bool>("draw-path", "Enable/Disable drawing ring in the final shoot", true);
	config.add<double>("canvas-border", "Free space around the canvas", 10);

	config.add<int>("dubins-resolution", "intial resolution of dop", 16);
	config.add<double>("dubins-radius", "radius of dubins vehicle", 0.5);
	config.add<bool>("save-targets", "disable/enable save targets", true);
	config.add<bool>("save-sampled-path", "disable/enable save sampled path", true);
	config.add<std::string>("targets", "file to save targets", "targets.txt");
	config.add<std::string>("sampled-path", "file to save sampled path", "sampled-path.txt");
	config.add<double>("sampled-path-distance", "distance between samples of path", 0.05);
	config.add<int>("num-iterations", "number of iteration used in VNS", 1000);
	config.add<int>("num-iterations-unimproved", "number of iteration used in VNS", 3000);
	config.add<bool>("use-rvns", "disable/enable randomized VNS", false);

	config.add<bool>("draw-neighborhood-points", "disable/enable draw points in neighborhoods", false);
	config.add<int>("maximal-calculation-time-sec", "maximal time for calculation", 600);

	config.add<bool>("lower-bound-above-budget-then-skip", "disable/enable skip when lowerbound above budget", true);

	config.add<bool>("generate-examples-of-operations", "disable/enable creation of example operations", false);
	config.add<double>("budget-override", "override budget with this value", 0);

	config.add<std::string>("planning-state", "planning state");
	config.add<int>("initial-prm-size");
	config.add<int>("extension-prm-size");
	config.add<bool>("save-after-optimized-length",
			"save/do-not-save samples and path after optimizion only the length", false);

	config.add<std::string>("gop-type", "type of GOP");
	config.add<std::string>("sop-solver", "type of GOP solver to use");

	config.add<bool>("initial-greedy-solution",
			"whether to generate greedy initial solution instead of none or start-goal solution", true);

	//opn
	config.add<double>("neighborhood-radius", "radius of neighborhood in OPN , DOPN ...", 0.0);
	config.add<int>("neighborhood-resolution", "radius of neighborhood in OPN , DOPN ...", 1);

	config.add<std::string>("map-type", "map-type is MAP_POINTS_CITY_POINTS blender or MAP_FILE");

	config.add<bool>("draw-targets-reward", "enable/disable drawing targets in different color using penalty", false);
	config.add<std::string>("draw-targets-reward-palette", "File name with colors for the reward palette", "");
	config.add<std::string>("draw-shape-targets", "Shape of the target",
			CShape("black", "green", 1, 4).getStringOptions());


	config.add<int>("random-seed","random seed to be used for srand, 0 means srand (time(NULL))", time(NULL));
	config.add<double>("collision-distance-check","distance in which the connection between samples is checked",0);
	config.add<double>("robot-size","size of robot for 2d problems",0);

	config.add<bool>("create-ilp-op-problem","if to create ilp definition after prm initialization",false);

	config.add<double>("maximal-acceleration", "maximal acceleration", 2.6);
	config.add<double>("fly-speed", "fly speed", 4.0);
	config.add<bool>("radius-from-acc-speed", "to calculate the radius from speed and acc", false);

	config.add<bool>("sample-by-time", "wether to sample by time and not by distance", false);
	config.add<double>("sample-time-s", "trajectory sample time in seconds", 0.2);
	config.add<double>("fly-altitude", "fly-altitude", 1.2);

	config.add<bool>("save-paths-during-optimization","save-paths-during-optimization",false);

	return config;
}

bool parseArgs(int argc, char *argv[]) {
	bool ret = true;
	std::string configFile;
	std::string guiConfigFile;
	std::string loggerCfg = "";

	po::options_description desc("General options");
	desc.add_options()("help,h", "produce help message")("config,c",
			po::value<std::string>(&configFile)->default_value(std::string(argv[0]) + ".cfg"), "configuration file")(
			"logger-config,l", po::value<std::string>(&loggerCfg)->default_value(loggerCfg),
			"logger configuration file")("config-gui",
			po::value<std::string>(&guiConfigFile)->default_value(std::string(argv[0]) + "-gui.cfg"),
			"dedicated gui configuration file")("problem", po::value<std::string>(&problemFile), "problem file");
	try {
		po::options_description guiOptions("Gui options");
		crl::gui::CGuiFactory::getConfig(guiConfig);
		crl::gui::CWinAdjustSize::getConfig(guiConfig);
		guiConfig.add<double>("gui-add-x",
				"add the given value to the loaded goals x coord to determine the canvas size and transformation", 0);
		guiConfig.add<double>("gui-add-y",
				"add the given value to the loaded goals y coord to determine the canvas size and transformation", 0);
		boost_args_add_options(guiConfig, "", guiOptions);
		guiOptions.add_options()("canvas-output", po::value<std::string>(&canvasOutput), "result canvas outputfile");

		po::options_description sopOptions("SOP solver options");
		//boost_args_add_options(SolverLP::getConfig(gopConfig), "", gopOptions);

		boost_args_add_options(addCommonConfig(gopConfig), "", sopOptions);

		po::options_description cmdline_options;
		cmdline_options.add(desc).add(guiOptions).add(sopOptions);

		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
		po::notify(vm);

		std::ifstream ifs(configFile.c_str());
		store(parse_config_file(ifs, cmdline_options), vm);
		po::notify(vm);
		ifs.close();
		ifs.open(guiConfigFile.c_str());
		store(parse_config_file(ifs, cmdline_options), vm);
		po::notify(vm);
		ifs.close();

		if (vm.count("help")) {
			std::cerr << std::endl;
			std::cerr << "Solver of Generalized Orienteering Problem ver. " << GOP_VERSION << std::endl;
			std::cerr << cmdline_options << std::endl;
			ret = false;
		}
		if (ret && loggerCfg != "" && fs::exists(fs::path(loggerCfg))) {
			std::cout << "creating logger with config....." << loggerCfg.c_str() << std::endl;
			crl::initLogger(LOGGER_NAME, loggerCfg.c_str());
			std::cout << "done" << std::endl;
		} else {
			std::cout << "creating logger without config....." << std::endl;
			logger = getLogger(LOGGER_NAME);

			std::cout << "created" << std::endl;
			log4cxx::LayoutPtr console_layout = new log4cxx::PatternLayout("%5p %d (%F:%L) - %m%n");

			std::cout << "console_layout created" << std::endl;
			log4cxx::AppenderPtr console_appender = new log4cxx::ConsoleAppender(console_layout);
			std::cout << "console_appender created" << std::endl;

			log4cxx::LayoutPtr file_layout = new log4cxx::PatternLayout("%5p %d (%F:%L) - %m%n");
			std::cout << "file_layout created" << std::endl;
			log4cxx::AppenderPtr file_appedner = new log4cxx::FileAppender(file_layout, "output.log");
			std::cout << "file_appedner created" << std::endl;

			//log4cxx::LevelPtr level = new log4cxx::Level();
			logger->setLevel(log4cxx::Level::getAll());
			std::cout << "setLevel done" << std::endl;
			logger->addAppender(console_appender);
			std::cout << "addAppender done" << std::endl;
			logger->addAppender(file_appedner);
			std::cout << "addAppender done" << std::endl;
			std::cout << "done" << std::endl;
		}
		if (!fs::exists(fs::path(problemFile))) {
			ERROR("Problem file '" + problemFile + "' does not exists");
			ret = false;
		}
	} catch (std::exception &e) {
		std::cerr << std::endl;
		std::cerr << "Error in parsing arguments: " << e.what() << std::endl;
		ret = false;
	}
	return ret;
}

CoordsVector &load_goals_coords(const std::string &filename, crl::CConfig &gopConfig, CoordsVector &pts) {
	Coords pt;
	std::string gop_type = gopConfig.get<std::string>("gop-type");
	gop_type = trim(gop_type);

	if (gop_type.compare("obst_op") == 0) {
		std::string map_type_str = gopConfig.get<std::string>("map-type");
		MapFormat map_format = op::VnsPrmOPSolver<HeapPoint3D>::parse_map_format(map_type_str);
		OP_Prolem<HeapPoint3D> loadedDataset;
		if (map_format == map_points_city_points) {
			loadedDataset = OBSTOPLoader::getSOPDefinition(gopConfig, filename);
		} else if (map_format == map_file) {
			loadedDataset = OBSTOPLoader::getSOPDefinitionMapFile(gopConfig, filename);
		}
		INFO("init loading received");
		int S = loadedDataset.map_points.size();
		for (int var = 0; var < S; ++var) {
			pts.push_back(Coords(loadedDataset.map_points[var].x, loadedDataset.map_points[var].y));
			INFO("map point x:"<<pts[var].x<< " y:"<<pts[var].y);
		}
		S = loadedDataset.border_nodes.size();
		for (int var = 0; var < S; ++var) {
			pts.push_back(Coords(loadedDataset.border_nodes[var]->data.x, loadedDataset.border_nodes[var]->data.y));
			INFO("border x:"<<pts[var].x<< " y:"<<pts[var].y);
		}
	} else {
		ERROR("unknown gop type "<<gop_type);
		exit(1);
	}
	return pts;
}

int main(int argc, char** argv) {
	Canvas *canvas = 0;
	int ret = -1;
	if (parseArgs(argc, argv)) {
		//log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger("lp_gop");

		//my_logger->addAppender()
		INFO("Start Logging");

		INFO_GREEN("seeding random")
		int random_seed = gopConfig.get<int>("random-seed");
		if (random_seed == 0) {
			INFO_GREEN("time(NULL) random seed")
			srand(time(NULL));
		} else {
			INFO_GREEN(random_seed<<" random seed")
			srand(random_seed);
		}
		try {
			CoordsVector pts;
			INFO("testing "<<problemFile);
			load_goals_coords(problemFile, gopConfig, pts);
			crl::gui::CWinAdjustSize::adjust(pts, guiConfig);
			INFO("seting gui");
			if (!is_displayable) {
				INFO_BLUE("disable gui - no displayable nodes");
				guiConfig.set<std::string>("gui", "none");
			}
			if ((g = gui::CGuiFactory::createGui(guiConfig)) != 0) {
				INFO("Start gui " + guiConfig.get<std::string>("gui"));
				canvas = new Canvas(*g);
			}

			char logName[40];

			std::string gop_solver = gopConfig.get<std::string>("sop-solver");
			if (gop_solver.compare("vns") == 0) {
				INFO("using vns solver");
				std::string planning_state_type_str = gopConfig.get<std::string>("planning-state");
				PlanningState planning_state = op::VnsPrmOPSolver<HeapPoint2D>::parse_planning_state(
						planning_state_type_str);
				if (planning_state == state2d) {
					INFO("planning for 2d -planning-state="<<planning_state_type_str);
					op::VnsPrmOPSolver<HeapPoint2D> vnssolver(gopConfig, problemFile);
					vnssolver.setCanvas(canvas);
					vnssolver.solve();
				} else if (planning_state == state2dheading) {
					INFO("planning for dubins 2d -planning-state="<<planning_state_type_str);
					op::VnsPrmOPSolver<HeapPoint2DHeading> vnssolver(gopConfig, problemFile);
					vnssolver.setCanvas(canvas);
					vnssolver.solve();
				} else if (planning_state == state3d) {
					INFO("planning for 3d -planning-state="<<planning_state_type_str);
					op::VnsPrmOPSolver<HeapPoint3D> vnssolver(gopConfig, problemFile);
					vnssolver.setCanvas(canvas);
					vnssolver.solve();
				} else {
					ERROR("unknown planning-state="<<planning_state_type_str);
					exit(1);
				}
			} else {
				ERROR("unknown sop-solver="<<gop_solver);
				ERROR("choose either vns or lp");
				exit(1);
			}

			INFO("after solve");
			usleep(100);
			if (canvas) {
				if (canvasOutput.size()) {
					INFO("canvas save");
					canvas->save(canvasOutput);
					INFO("canvas save end");
				}
				if (!guiConfig.get<bool>("nowait")) {
					INFO("click to exit");
					canvas->click();
					usleep(100);
				}
				INFO("delete canvas");
				delete canvas;
				delete g;
			}
			//std::list<GraphNode> foundTour = fph.find(nodes, budget, startIndex, goalIndex);
			//Tour foundTour = fph.find(loadedDataset);

			//INFO("tour founded");
		} catch (crl::exception &e) {
			ERROR("Exception " << e.what() << "!");
		} catch (std::exception &e) {
			ERROR("Runtime error " << e.what() << "!");
		}
		ret = EXIT_SUCCESS;
	}
	crl::shutdownLogger();
	return ret;
}

