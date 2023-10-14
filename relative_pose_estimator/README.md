# Relative Pose Estimator
This package computes two poses:
* received UAV pose relative to (0,0) point in global map 
* received UAV pose relative to the UAV that is merging maps

The speed at which this node publishes the relative poses can be changed in config files with variable rate/publish_relative_pose.

Main reason for spliting the merging process and pose approximation process into two is that we does not have to use threads in one nodelet to run simultaneously both task. We can instead divide the tasks into two nodelets that can work simultaneously.
