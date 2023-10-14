# The Map Merging for UAV Swarms SOURCE CODE

## Structure
This package consists of:

### Experiments
Folder containing many experiments used to validate the functionality of this package. This folder is divided into `real_world` experiments and the experiments in a `simulation` environment.
This folder also contains `worlds` file with a script for creation of diverse simulation environments. To use the script, just start the *start.sh* script in the directory /experiments/worlds/gazebo_world_creator. 
Unfortunatelly, any rosbag files used for testing had to be removed before submission due to their large size.

### Occupancy grid merger nodelet
This is the main algorithm used for merging the occupancy grid maps. The main .cpp files are:
* `OccupancyGridMerger.cpp`: Which is the main nodelet which publish the merged occupancy grid map and the pose of the received map relative to the current map.
* `Merger2d.cpp`: Class for merging occupancy grid maps.
* `Evolution.cpp`: The main heart of this algorithm. It contains the evolution methods inspired by the genetic algorithm.
* `Generation.cpp`: Each iteration of evolution produces new generation which is capsulated in the Generation class defined in this file.
* `Population.cpp, Individual.cpp`: Files used in the generation class for storing population and ofcourse as the name suggest each individual.
* `transformations.cpp`: File with functions used to transform many different things like pose, point, OpenCV image, and quaternions.

There is also a `config` folder containing all optional parameters that can be changed also during the runtime of the map merging process.

### Relative pose nodelet
This is another nodelet that can be used for fast approximation of the pose of the other UAVs.
You can set again in the `config` folder of this nodelet the rate of timer used for approximating the position. 
This nodelet publish the pose of other UAV relative to the (0,0) coordinate in the map of the UAV that is merging maps (with the occupancy_grid_merger nodelet).
It also publishes the pose of other UAV relative to the pose of currently merging UAV.

## Installation of required software
The MRS group uav system and HectorSLAM has to be installed before the code can be run. Fortunatelly the mrs_uav_system code was recently made available as a free source on [github](https://github.com/ctu-mrs/mrs_uav_system). Please, follow the installation instructions on the github page.
After the successful installation of mrs_uav_system, copy and paste the ROS nodes occupancy_grid_merger and relative_pose into the your newly created workspace and build them by using `catkin build` command. the HectorSLAM package also has to be installed. For that, follow the instructions on [MRS wiki page about HectorSLAM.](https://ctu-mrs.github.io/docs/software/hector_slam.html)

## Launching the Occupancy grid merger
First I highly reccomend reading through the [MRS github documentation](https://ctu-mrs.github.io/docs/simulation/howto.html) where you can learn everything necessary about the usage of this package. 

There are two primary means of use for this package:
### Simulation
To use this package as a simulation the launch files named *launch.simulation* should be used. You can either run a gazebo simulation which is very demanding on performance. Or a rosbag files can be used. If you have no usable data then the bagfiles can be recorded in the gazebo simulation for example by starting the *start.sh* script in the folder /experiments/maze/one_drone_maze and by performing two or more mapping processes. After the bagfiles are recorded (and sometimes reindexed) they can be run by inserting their /path/to/rosbag.bag into the *session.yml* file in folder /experiments/maze/test_maze_bagfiles and running the start.sh script in the same folder.

### Real-World Experiment
For real world experiments with more UAVs the Nimbro Network should be used for communication between the UAVs. 

### Issues
Some testing files could contain an absolute path from my computer. If you see something like the /home/maderja1/workspace/src/... it is my absolute path. Feel free to change it.

### ROSDOC
The rosdoc_lite can be used on rosdoc.yaml files in occupancy_grid_merger and relative_pose packages to generate documentation for better readability of function and methods descriptions.
