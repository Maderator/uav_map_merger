/**
 * @file OccupancyGridMerger.h
 * @author Jan Maděra <maderja@fel.cvut.cz> (janmadera97@gmail.com)
 * @version 0.0.1
 * @date 2019-12-02
 */

#pragma once
#ifndef OCCUPANCY_GRID_MERGER_H
#define OCCUPANCY_GRID_MERGER_H

/* includes //{ */

/* my includes */
#include "Merger2d.h"
#include "transformations.h"
#include "types.h"

/* ROS includes for data transformation between ROS and OpenCV */
#include <cv_bridge/cv_bridge.h>

#include <string> 
#include <string.h>

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for loading occupancy map from rosbag */
#include <rosbag/bag.h>
#include <rosbag/view.h>

/* for loading dynamic parameters while the nodelet is running */
#include <dynamic_reconfigure/server.h>

/* this header file is created during compilation from python script dynparam.cfg */
#include <occupancy_grid_merger/dynparamConfig.h>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for visualization of poses and occupied cells */
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

/* for storing information abotu the state of the uav (occupancy grid map) */
#include <nav_msgs/OccupancyGrid.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom msgs of MRS group */
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>

//}

namespace occupancy_grid_merger
{

/* class OccupancyGridMerger //{ */
class OccupancyGridMerger : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_ = false;

  /* ros parameters */
  bool _gui_ = true;

  bool        _simulation_;
  std::string _uav_name_;

  // | ---------------------- msg callbacks --------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_rec1_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_rec2_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_ground_truth_;
  
  // My handlers
  mrs_lib::SubscribeHandler<nav_msgs::OccupancyGrid>             sh_rec_og_; //!< received occupancy grid
  std::mutex                       mutex_rec_og_;

  mrs_lib::SubscribeHandler<nav_msgs::OccupancyGrid>             sh_uav_og_;
  std::mutex                       mutex_uav_og_;

  // | --------------------- timer callbacks -------------------- |
 
  // muj callback
  void           callbackTimerPublishOccupancyMapMerging(const ros::TimerEvent& te);
  ros::Timer     timer_publish_occupancy_map_merging_;
  float            _rate_timer_publish_occupancy_map_merging_;
  
  void           callbackTimerPublishRelativePose(const ros::TimerEvent& te);
  ros::Timer     timer_publish_relative_pose_;
  float           _rate_timer_publish_relative_pose_;
  bool           _pub_rel_pose_first_time_ = true;
  geometry_msgs::PoseStamped _rel_map_pose_;

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | ---------------- service server callbacks ---------------- |
  
  bool               callbackTest([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_test_;

  // | ----------------- map merging publishers ----------------- |
  ros::Publisher pub_og_map_new_uav_;
  ros::Publisher     pub_og_map_merged_;
  ros::Publisher     pub_relative_map_pose_; //!< publisher of Received Map relative pose to this UAV map
  
  // TOPICS USED FOR VISUALIZATION
  ros::Publisher pub_occupied_cells_rec_; //!< publisher of occupied cells of received map transformed by relative pose with highest map consistency
  ros::Publisher pub_occupied_cells_cur_; //!< publisher of occupied cells of current map transformed by relative pose with highest map consistency
  ros::Publisher pub_consistent_cells_; //!< publisher of cells that are occupied in both occupancy grid maps and are consistent (covering each other)
  ros::Publisher pub_generation_poses_; //!< publisher of array of poses in each generation of evolution

  // | -------------- changing acceptance threshold ------------- |
  std::mutex mutex_merger_idle_time_; // merger s not merging... we can change acceptance threshold

  // | ------------------- dynamic reconfigure ------------------ |

  typedef occupancy_grid_merger::dynparamConfig                              Config;
  typedef dynamic_reconfigure::Server<occupancy_grid_merger::dynparamConfig> ReconfigureServer;
  boost::recursive_mutex                                              mutex_dynamic_reconfigure_;
  boost::shared_ptr<ReconfigureServer>                                reconfigure_server_;
  void                                                                callbackDynamicReconfigure(Config& config, uint32_t level);
  occupancy_grid_merger::dynparamConfig                                      last_drs_config_;

  // | -------------------- merging variables ------------------- |
  merger2d::Merger2d _merger_;
  bool               _first_time_merging_ = true;
  TimerMergingInfo _merging_info_;

  // | ------------------- merging parameters ------------------- |
  float _acceptance_threshold_;
  float _occupancy_threshold_;
  int _population_size_;
  int  _min_micro_sec_;
  int  _max_micro_sec_;
  float _err_range_;
  float _err_angle_;
  bool _starting_close_;

  // | -------------------- support functions ------------------- |


  void copyOccupancyGridInfoAndHeader(nav_msgs::OccupancyGridPtr dest, nav_msgs::OccupancyGridConstPtr src);

};
//}

}  // namespace occupancy_grid_merger
#endif
