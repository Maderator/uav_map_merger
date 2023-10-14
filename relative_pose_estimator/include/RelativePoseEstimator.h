#pragma once
#ifndef RELATIVE_POSE_ESTIMATOR_H
#define RELATIVE_POSE_ESTIMATOR_H

/* includes //{ */

// My includes
#include "transformations.h"

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <string>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/OccupancyGrid.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>

//}

namespace relative_pose_estimator
{

/* class RelativePoseEstimator //{ */
class RelativePoseEstimator : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_ = false;

  /* ros parameters */
  bool        _simulation_;
  std::string _uav_name_;

  // | ---------------------- msg callbacks --------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::OccupancyGrid>             sh_cur_occup_map_;
  mrs_lib::SubscribeHandler<nav_msgs::OccupancyGrid>             sh_rec_occup_map_;
  mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>          sh_rel_map_pose_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_rec1_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_ground_truth_;
  
  // | --------------------- timer callbacks -------------------- |
 
  void           callbackTimerPublishRelativePose(const ros::TimerEvent& te);
  ros::Timer     timer_publish_relative_pose_;
  float           _rate_timer_publish_relative_pose_;
  bool           _pub_rel_pose_first_time_ = true;
  geometry_msgs::PoseStamped _glob_rec_pose_;
  geometry_msgs::PoseStamped _rel_rec_pose_;

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | ----------------- map merging publishers ----------------- |
  ros::Publisher     pub_relative_pose_; //!< publisher of other UAV relative pose relative to this UAV
  ros::Publisher     pub_relative_pose_in_glob_map_; //!< publisher of relative pose of other UAV relative to the global map

};
//}

}  // namespace relative_pose_estimator
#endif
