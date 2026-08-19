# UAV Map Merger

> **Status: historical research code.** This repository contains the source code accompanying Jan Madera's bachelor thesis at the Faculty of Electrical Engineering, Czech Technical University in Prague. The original thesis snapshot is preserved in the [`thesis-submission` release](https://github.com/Maderator/uav_map_merger/releases/tag/thesis-submission). The project is now being documented and reviewed for preservation and reuse, but it is **not currently a verified or supported installation**.

## Overview

This project explores map merging for UAV swarms. Its primary ROS nodelet aligns and merges occupancy-grid maps, using an evolutionary search to estimate the relative transformation between them. A second nodelet estimates the relative pose of the UAVs to help initialise and interpret the merge.

The repository contains:

- `occupancy_grid_merger` — the primary ROS nodelet and map-merging algorithm.
- `relative_pose_estimator` — a ROS nodelet for relative-pose estimation.
- `experiments` — simulation and real-world experiment setups and evaluation data.
- [Bachelor thesis (PDF)](bachelor_thesis_text.pdf) — the accompanying thesis.

## How it works

The occupancy-grid merger:

1. receives a local occupancy grid and a second grid to merge;
2. searches for the relative 2D translation and rotation using an evolutionary algorithm;
3. publishes the merged occupancy grid and the estimated relative pose.

The main implementation areas are:

- `OccupancyGridMerger.cpp` — ROS nodelet orchestration, inputs, and outputs.
- `Merger2d.cpp` — occupancy-grid merging.
- `Evolution.cpp` — evolutionary search.
- `Generation.cpp`, `Population.cpp`, and `Individual.cpp` — evolutionary population management.
- `transformations.cpp` — transformations between ROS, OpenCV, poses, points, and quaternions.

Runtime parameters are available in the packages' `config` directories and use ROS dynamic reconfigure where applicable.

## Historical environment

The source declares or uses the following original requirements:

- ROS 1 and Catkin
- C++17
- OpenCV
- MRS UAV System libraries and messages (`mrs_lib`, `mrs_msgs`)
- Hector SLAM / `hector_map_tools`
- Gazebo and `mrs_simulation` for simulation experiments
- ROS packages including `nodelet`, `geometry_msgs`, `nav_msgs`, `cv_bridge`, `image_transport`, and `dynamic_reconfigure`
- Nimbro Network for communication in multi-UAV real-world experiments

The exact Ubuntu release, ROS distribution, compiler version, OpenCV version, and MRS/Hector revisions were not recorded in the repository. Consequently, this is **not yet a reproducible setup guide**. A clean, pinned environment must be established and verified before any platform is described as supported.

## Support policy

This repository currently preserves an academic implementation rather than a maintained robotics product.

- The original code has not yet been rebuilt in a clean environment.
- Experiment files contain machine-specific paths and require adaptation.
- Original ROS bag files are not included because of their size.
- Simulation and real-world experiments depend on the surrounding MRS UAV ecosystem.

When a verified build and a minimal demonstration are available, this section will be updated with the supported platform and reproduction instructions.

## Repository layout

### `occupancy_grid_merger`

The main map-merging nodelet. It publishes a merged occupancy grid and the estimated relative pose of the received map.

### `relative_pose_estimator`

A nodelet that estimates other UAVs' poses relative to the map origin and to the UAV performing the merge.

### `experiments`

Simulation and real-world validation setups, Gazebo worlds, plotting utilities, and experimental results. The Gazebo world creator can be found in `experiments/worlds/gazebo_world_creator`.

## Legacy usage notes

The original workflow expected both ROS packages to be placed in an existing MRS workspace and built with `catkin build`. Refer to the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) and [Hector SLAM documentation](https://ctu-mrs.github.io/docs/software/hector_slam.html) for the upstream systems.

Some experiment files contain paths from the original development machine, such as `/home/maderja1/...`. These paths must be replaced with locations appropriate to the user's workspace. The experiment launch scripts and session files should therefore be treated as historical examples until they are cleaned and verified.

## Documentation

Each ROS package includes a `rosdoc.yaml` file. `rosdoc_lite` can be used to generate API documentation from the source comments.

## Maintenance roadmap

Current maintenance work focuses on:

1. preserving the original thesis snapshot;
2. identifying and reproducing the historical build environment;
3. removing machine-specific paths;
4. adding a small reproducible demonstration, tests, and continuous integration.

## License

This repository is licensed under the [BSD 3-Clause License](LICENSE). The package-level license files preserve the original copyright notices for their respective packages.
