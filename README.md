# Street-level Simulator for SCENTS(collaborative sensing in proximity IoT networks)
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

## Getting Started

**(Jan 2020) Updated the simulator to work with OMNeT(v5.6) and INeT(v4.2)**

(Oct 2019) Updated the simulator to work with OMNeT(v5.5.1) and INeT(v4.1.1).

### Prerequisites
- [OMNeT++ v5.6](https://omnetpp.org/download/)

- [INeT Framework v4.2](https://github.com/inet-framework/inet)

- [Eigen3](http://eigen.tuxfamily.org) C++ template library for linear algebra.

- Include the 3rd party libraries in project path (shown in below fig)
<img src="figures/include.png" width="450">

### Experiment Scenarios

Config: `simulations/scents/omnetpp.ini`
Run Script: `simulations/scents/run.sh`

### Customize the simulation

You can download the data of your target geographical region in OSM
format and convert the buildings using the osm parser
script. Waypoints used in the mobility model are also GPS
coordinates. Other things are mostly configurable through
`omnetpp.ini`.

## Screenshots

<img src="figures/west_austin_scene.png" width="600">

[(YouTube vid)](https://youtu.be/KPqtK9t2efs)


## License and Citation
This simulator is released under the BSD 3-Clause license.

Please use the following reference in your publications if this project helps your research:
```
@inproceedings{liu2019scents,
  title={SCENTS: Collaborative Sensing in Proximity IoT Networks},
  author={Liu, Chenguang and Hua, Jie and Julien, Christine},
  booktitle={Proceedings of the 17th IEEE International Conference on Pervasive Computing and Communications Workshops (PerCom Workshops)},
  year={2019},
  organization={IEEE},
}
```
