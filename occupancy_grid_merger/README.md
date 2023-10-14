# Occupancy Grid Merger

This package was created as a part of a bachelor thesis called Map Merging for UAV Swarms. Its main functionality is merging multiple occupancy grid maps from several UAVs to a single map and approximation of UAVs relative positions. The package is written in C++.

## Functionality
* Please, use the timer publisher as the main method for occupancy grid map merging. The service methods were used in testing phase of the algorithm implementation and should be used only with caution that there is a high probability of error if the services are used.
* Service `merge_occupancy_grid_map` estimates relative pose between the two maps and merges them. Publish the occupancy grid map message on topic occupancy_grid_merging/merged_map
* Service `test` was used for testing of functionality of the occupancy grid merger

## Package structure

See [ROS packages](http://wiki.ros.org/Packages)

* `src` directory contains all source files
* `include` directory contains all header files.
* `launch` directory contains `.launch` files which are used to parametrize the nodelet. Command-line arguments, as well as environment variables, can be loaded from the launch files, the nodelet can be put into the correct namespace (each UAV has its namespace to allow multi-robot applications), config files are loaded, and parameters passed to the nodelet. See [.launch files](http://wiki.ros.org/roslaunch/XML)
* `config` directory contains parameters in `.yaml` files. See [.yaml files](http://wiki.ros.org/rosparam)
* `package.xml` defines properties of the package, such as package name and dependencies. See [package.xml](http://wiki.ros.org/catkin/package.xml)
