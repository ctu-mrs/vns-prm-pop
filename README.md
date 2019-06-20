## VNS-PRM* for Physical Orienteering Problem

#### The source codes will be provided after publication of the proposed method. 

This repository will contain solver for the Physical Orienteering Problem (POP) based on the Variable Neighborhood Search (VNS) method combined with the asymptotically optimal sampling-based Probabilistic Roadmap (PRM*).

**Repository contains following content:**
- **datasets** folder contains dataset instances of POP
- **results** folder contains computational results shown in the [paper](https://doi.org/10.1109/LRA.2019.2923949)
- **sources** folder contains source codes for VNS-PRM* solver for POP
- **visualization** folder contains python scripts for visualizing the computed results 

### Dependencies 

For Ubuntu 18.04 LTS the dependencies can be installed using apt as:
```bash 
sudo apt-get install make ccache build-essential pkg-config liblog4cxx-dev libcairo2-dev libboost-filesystem-dev libboost-program-options-dev libboost-thread-dev libboost-iostreams-dev libboost-system-dev
```

### Cloning and compilation of supporting libraries

To clone the repository run
```bash 
git clone --recursive https://github.com/ctu-mrs/vns-prm-pop.git
```
and compile supporting libraries by running
```bash 
cd vns-prm-pop/sources
make dependencies
```

### Compilation

The VNS-PRM* can be compiled using the above dependencies by running **make** in folder **vns-prm-pop/sources***. 

### Running VNS-PRM*

After compilation, the VNS-SOP can be run using **sop\_vns** program and the ILP solver for SOP can be run using **sop\_ilp** program.
The default configuration of programs is stored in **sop.cfg** file. 
The most important configuration parameters are:
- _problem_ - specifies location of sop dataset instance file
- _gui_ - switch between "cairo" gui, "none" gui and "cairo-nowin" with background creation of gui images (gui is only shown for pure SOP and OPN instances, the DOP instances have no gui in runtime, however, can be visualized using the show_solution.py script)
- _nowait_ - switch whether to close gui window after finish of the solver
- _planning-state_ - switch between "2d", "dubins2d" or "3d" planning states
- _map-type_ - switch between "MAP_POINTS_CITY_POINTS" for potholes and dense scenario, and "MAP_FILE" for building scenario
- _collision-distance-check_ - distance in which check distance, aproximately minimal width among obstacles

The configuration parameters can be also set as a command line parameters, e.g. by running

```bash
./vns_prm_pop --problem=../datasets/potholes/potholes-cell.txt --map-type=MAP_POINTS_CITY_POINTS --planning-state=2d --collision-distance-check=2.0 --gui=cairo --nowait=0
```

or

```bash
./vns_prm_pop --problem=../datasets/dense/dense-cell.txt --map-type=MAP_POINTS_CITY_POINTS --planning-state=2d --collision-distance-check=2.0 --gui=cairo --nowait=0
```

or

```bash
./vns_prm_pop --problem=../datasets/building/building.txt --map-type=MAP_FILE --planning-state=3d --collision-distance-check=0.3 --gui=cairo --nowait=0
``` 


### Visualization



