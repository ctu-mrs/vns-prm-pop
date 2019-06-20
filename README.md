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

### Visualization



