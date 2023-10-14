/**  @file
 *   @brief Contains definition of nodelet OccupancyGridMerger
 *   @author Jan Maděra <maderja@fel.cvut.cz> (janmadera97@gmail.com)
 *   @date 2020
 *   @copyright The 3-Clause BDS License
 */

/* include header file of this class */
#include "OccupancyGridMerger.h"

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

/* ROS includes for data transformation between ROS and OpenCV */
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <string.h>

using namespace cv;
using namespace std;

namespace occupancy_grid_merger
{

/* onInit() //{ */

void OccupancyGridMerger::onInit() {

  ROS_INFO("Initializing occupancygridmerger node");
  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here

  // insert booleans

  /* obtain node handle */
  ros::NodeHandle nh("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader param_loader(nh, "OccupancyGridMerger");

  param_loader.loadParam("UAV_NAME", _uav_name_);
  param_loader.loadParam("gui", _gui_);
  param_loader.loadParam("simulation", _simulation_);
  param_loader.loadParam("rate/publish_occupancy_grid_map_merging", _rate_timer_publish_occupancy_map_merging_);
  param_loader.loadParam("rate/publish_relative_pose", _rate_timer_publish_relative_pose_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("acceptance_threshold", _acceptance_threshold_);
  param_loader.loadParam("occupancy_threshold", _occupancy_threshold_);
  param_loader.loadParam("population_size", _population_size_);
  param_loader.loadParam("min_micro_sec", _min_micro_sec_);
  param_loader.loadParam("max_micro_sec", _max_micro_sec_);
  param_loader.loadParam("err_range", _err_range_);
  param_loader.loadParam("err_angle", _err_angle_);
  param_loader.loadParam("starting_close", _starting_close_);
  

  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _acceptance_threshold_ << " acceptance threshold loaded");
  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _occupancy_threshold_ << " occupancy threshold loaded");
  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _population_size_ << " population size loaded");
  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _min_micro_sec_ << " minimum evolution runtime in microseconds loaded");
  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _max_micro_sec_ << " maximum evolution runtime in microseconds loaded");
  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _err_range_ << " maximal error of position of relative pose loaded");
  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _err_angle_ << " maximal error angle loaded");
  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _starting_close_ << " boolean true if UAVs start close together loaded");
  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _rate_timer_publish_occupancy_map_merging_ << " publish rate of merging occupancy grid in Hz.");
  ROS_INFO_STREAM_ONCE("[OccupancyGridMerger]: " << _rate_timer_publish_relative_pose_ << " publish rate of computing relative pose in Hz.");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[OccupancyGridMerger]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ------------------ initialize subscribers ----------------- |
  
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "OccupancyGridMerger";
  shopts.no_message_timeout = ros::Duration(5.0);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odometry_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_uav_in");
  sh_odometry_rec1_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_rec1_in");
  sh_odometry_rec2_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_rec2_in");
  sh_rec_og_ = mrs_lib::SubscribeHandler<nav_msgs::OccupancyGrid>(shopts, "rec_map1_in");
  sh_uav_og_ = mrs_lib::SubscribeHandler<nav_msgs::OccupancyGrid>(shopts, "occupancy_grid_map_in");

  /* subscribe ground truth only in simulation, where it is available */
  if (_simulation_) {
    sh_ground_truth_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_gt_in");
  }

  pub_relative_map_pose_ = nh.advertise<geometry_msgs::PoseStamped>("relative_map_pose_out", 1);
  pub_og_map_new_uav_ = nh.advertise<nav_msgs::OccupancyGrid>("og_map_new_out", 1); //!< second parameter is publisher queue size of outgoing messages
  pub_og_map_merged_ = nh.advertise<nav_msgs::OccupancyGrid>("og_map_merged_out", 1);
  pub_occupied_cells_rec_ = nh.advertise<visualization_msgs::Marker>("occupied_cells_rec_out", 1);
  pub_occupied_cells_cur_ = nh.advertise<visualization_msgs::Marker>("occupied_cells_cur_out", 1);
  pub_consistent_cells_ = nh.advertise<visualization_msgs::Marker>("consistent_cells_out", 1);
  pub_generation_poses_ = nh.advertise<geometry_msgs::PoseArray>("generation_poses_out", 1);

  // | -------------------- initialize timers ------------------- |
 
  timer_publish_occupancy_map_merging_    = nh.createTimer(ros::Rate(_rate_timer_publish_occupancy_map_merging_), &OccupancyGridMerger::callbackTimerPublishOccupancyMapMerging, this);

  timer_publish_relative_pose_    = nh.createTimer(ros::Rate(_rate_timer_publish_relative_pose_), &OccupancyGridMerger::callbackTimerPublishRelativePose, this);

  timer_check_subscribers_        = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &OccupancyGridMerger::callbackTimerCheckSubscribers, this);

  // | --------------- initialize service servers --------------- |
  
  srv_server_test_ = nh.advertiseService("test_in", &OccupancyGridMerger::callbackTest, this);

  // | ---------- initialize dynamic reconfigure server --------- |
 
  reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh));
  ReconfigureServer::CallbackType f = boost::bind(&OccupancyGridMerger::callbackDynamicReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  /* set the default value of dynamic reconfigure server to the value of parameter with the same name */
  {
    std::scoped_lock lock(mutex_merger_idle_time_);
    last_drs_config_.acceptance_threshold = _acceptance_threshold_;
    last_drs_config_.occupancy_threshold = _occupancy_threshold_;
    last_drs_config_.population_size = _population_size_;
    last_drs_config_.min_micro_sec = _min_micro_sec_;
    last_drs_config_.max_micro_sec = _max_micro_sec_;
    last_drs_config_.err_range = _err_range_;
    last_drs_config_.err_angle = _err_angle_;
    last_drs_config_.starting_close = _starting_close_;
  }
  reconfigure_server_->updateConfig(last_drs_config_);

  // | ----------------- initialize merging info ---------------- |
  _pub_rel_pose_first_time_ = true;
  _first_time_merging_ = true;
  _merging_info_.seq = 0;
  _merging_info_.previous_rel_pose = boost::make_shared<geometry_msgs::Pose>();
  _merging_info_.has_rel_pose = false;

  ROS_INFO_ONCE("[OccupancyGridMerger]: initialized");

  is_initialized_ = true;
}
//}

// | --------------------- timer callbacks -------------------- |
/* callbackTimerPublishOccupancyMapMerging() //{ */
void OccupancyGridMerger::callbackTimerPublishOccupancyMapMerging([[maybe_unused]] const ros::TimerEvent& te){

  /* CHECKS, WARNINGS //{ */
  
  if (!is_initialized_) {
    ROS_WARN_THROTTLE(1.0, "[OccupancyGridMerger]: Timer cannot merge occupancy grid map, nodelet is not initialized.");
    return;
  }
  
  if(!sh_uav_og_.hasMsg()){  
    ROS_WARN_THROTTLE(1.0, "[ccupancyGridMerger]: Timer cannot merge occupancy grid map, occupancy grid of this UAV not yet received.");
    return;
  }
  
  if(!sh_rec_og_.hasMsg()){  
    ROS_WARN_THROTTLE(1.0, "[ccupancyGridMerger]: Timer cannot merge occupancy grid map, occupancy grid of another UAV not yet received.");
    return;
  }
  
  if(!sh_odometry_.hasMsg()){
    ROS_WARN_THROTTLE(1.0, "[OccupancyGridMerger]: Cannot merge occupancy grid map, odometry not yet recieved.");
    return;
  }
  
  if(!sh_odometry_rec1_.hasMsg()){
    ROS_WARN_THROTTLE(1.0, "[OccupancyGridMerger]: Cannot merge occupancy grid map, odometry from other UAV not yet recieved.");
    return;
  }
  
  //}

  // recieved og
  nav_msgs::OccupancyGridPtr uav_og_p = boost::make_shared<nav_msgs::OccupancyGrid>();
  nav_msgs::OccupancyGridPtr rec_og_p = boost::make_shared<nav_msgs::OccupancyGrid>();
  // copy pose from odometry of current uav
  geometry_msgs::PosePtr odom_pose_p = boost::make_shared<geometry_msgs::Pose>(sh_odometry_.getMsg()->pose.pose);
  geometry_msgs::PosePtr odom_pose_rec_p = boost::make_shared<geometry_msgs::Pose>(sh_odometry_rec1_.getMsg()->pose.pose);

  double cur_yaw = merger2d::getYaw(odom_pose_p->orientation);
  double rec_yaw = merger2d::getYaw(odom_pose_rec_p->orientation);
  if (isnan(cur_yaw) || isnan(rec_yaw)) {
    ROS_WARN("[OccupancyGridMerger]: Pose orientation of one of the UAVs is not a number!!!");
    return;
  }

  { 
    std::scoped_lock lock(mutex_uav_og_);
    nav_msgs::OccupancyGridConstPtr uav_og_msg = sh_uav_og_.getMsg();
    uav_og_p->data = uav_og_msg->data;
    uav_og_p->header = uav_og_msg->header;
    uav_og_p->info = uav_og_msg->info;
  }

  {
    std::scoped_lock lock(mutex_rec_og_);
    nav_msgs::OccupancyGridConstPtr rec_og_msg = sh_rec_og_.getMsg();
    rec_og_p->data = rec_og_msg->data;
    rec_og_p->header = rec_og_msg->header;
    rec_og_p->info = rec_og_msg->info;
  }

  // --------------------------------------------------------------
  // |                        MERGING BLOCK                       |
  // --------------------------------------------------------------

  nav_msgs::OccupancyGridPtr merged_map;
  {  
      std::scoped_lock lock(mutex_merger_idle_time_);
    EvolutionParams params = {
      .accept_thr = &_acceptance_threshold_, 
      .occup_thr = &_occupancy_threshold_, 
      .pop_size = &_population_size_,
      .gui = &_gui_,
      .min_mic_sec = &_min_micro_sec_,
      .max_mic_sec = &_max_micro_sec_,
      .err_range = &_err_range_,
      .err_angle = &_err_angle_,
      .starting_close = _starting_close_,
      .pub_rel_pose = pub_relative_map_pose_,
			.pub_occup_cells_rec = pub_occupied_cells_rec_,
			.pub_occup_cells_cur = pub_occupied_cells_cur_,
      .pub_consistent_cells = pub_consistent_cells_,
			.pub_gen_poses = pub_generation_poses_,
      .rec_og_frame_id = rec_og_p->header.frame_id,
      .uav_og_frame_id = uav_og_p->header.frame_id,
      .rec_og_stamp = rec_og_p->header.stamp,
      .uav_og_stamp = uav_og_p->header.stamp,
      .rec_og_origin = rec_og_p->info.origin,
      .uav_og_origin = uav_og_p->info.origin,
      .rec_og_resolution = rec_og_p->info.resolution,
      .uav_og_resolution = uav_og_p->info.resolution
    };

    if(!_first_time_merging_){
      _merger_.setEvolutionParams(params);
      _merger_.setMemberVars(uav_og_p, rec_og_p);
      _merger_.initializeEvolver();
      _merger_.setLocalPoses(odom_pose_p, odom_pose_rec_p);
    } else{
      //verification::initCSVMapFile("/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/maze_opposite_45/merging1.csv");
      _merger_ = merger2d::Merger2d(uav_og_p, rec_og_p, odom_pose_p, odom_pose_rec_p, params);
      _merger_.initializeEvolver();
      _merger_.initializeGAGeneration();
      _first_time_merging_ = false;
    }
    merged_map = _merger_.mergeMapsContinuously();

    merged_map->header.seq = _merging_info_.seq;  
    ++_merging_info_.seq;
    _merging_info_.previous_rel_pose = boost::make_shared<geometry_msgs::Pose>(*_merger_.getRelativeMapPose());
    _merging_info_.has_rel_pose = true;
    _merging_info_.prev_loc_cur_p = boost::make_shared<geometry_msgs::Pose>(*odom_pose_p);
    _merging_info_.prev_loc_rec_p = boost::make_shared<geometry_msgs::Pose>(*odom_pose_rec_p);
    _merging_info_.previous_acceptance_index = _merger_.getAcceptanceIndex();
  }
  // MERGING BLOCK END

  try {
    pub_og_map_merged_.publish(merged_map);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_og_map_merged_.getTopic().c_str());
  }

  return;
}
//}

/*  callbackTimerPublishRelativePose() //{ */
void OccupancyGridMerger::callbackTimerPublishRelativePose([[maybe_unused]] const ros::TimerEvent& te){
  
  /* CHECKS, WARNINGS //{ */
  if (!is_initialized_) {
    return;
  }
  
  if(!_merging_info_.has_rel_pose){
    ROS_WARN_THROTTLE(2.0, "[OccupancyGridMerger]: Relative map pose not yet computed.");
    return;
  }
  //}

  if(!_pub_rel_pose_first_time_){
    ++_rel_map_pose_.header.seq;
  } else {
    _rel_map_pose_.header.seq = 0;
    _rel_map_pose_.header.frame_id = sh_odometry_.getMsg()->header.frame_id;
    _pub_rel_pose_first_time_ = false;
  }
  _rel_map_pose_.header.stamp = ros::Time().now();
  
  _rel_map_pose_.pose = *_merger_.getRelativeMapPose();
  try {
    pub_relative_map_pose_.publish(_rel_map_pose_);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_relative_map_pose_.getTopic().c_str());
  }
  return;
}

//}

/* callbackTimerCheckSubscribers() //{ */

void OccupancyGridMerger::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  if (!sh_odometry_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "Not received uav odom msg since node launch.");
  }
  
  if (!sh_odometry_rec1_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "Not received rec uav odom msg since node launch.");
  }

  if (!sh_rec_og_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "Not received occupancy grid of other uav msg since node launch.");
  }

  if (!sh_uav_og_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "Not received uav occupancy grid msg since node launch.");
  }

  if (_simulation_) {
    if (!sh_ground_truth_.hasMsg()) {
      ROS_WARN_THROTTLE(10.0, "Not received ground truth odom msg since node launch.");
    }
  }
}
//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackTest() */
bool OccupancyGridMerger::callbackTest([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res){
  if (!is_initialized_) {
    res.success = false;
    res.message = "Occupancy Grid Map Merge not initialized!";
    ROS_WARN("[OccupancyGridMerger]: Cannot run test, nodelet is not initialized.");
    return true;
  }

  if(!sh_uav_og_.hasMsg()){  
    res.success = false;
    res.message = "Occupancy grid of this UAV not yet recieved!";
    ROS_WARN("[ccupancyGridMerger]: Cannot merge occupancy grid map, occupancy grid of this UAV not yet received.");
    return true;
  }

  if(!sh_rec_og_.hasMsg()){  
    res.success = false;
    res.message = "Occupancy grid of another UAV not yet recieved!";
    ROS_WARN("[ccupancyGridMerger]: Cannot merge occupancy grid map, occupancy grid of another UAV not yet received.");
    return true;
  }

  if(!sh_odometry_.hasMsg()){
    res.success = false;
    res.message = "Odometry not yet recieved!";
    ROS_WARN("[OccupancyGridMerger]: Cannot merge occupancy grid map, odometry not yet recieved.");
    return true;
  }

  if(!sh_odometry_rec1_.hasMsg()){
    res.success = false;
    res.message = "Odometry not yet recieved!";
    ROS_WARN("[OccupancyGridMerger]: Cannot merge occupancy grid map, odometry from other UAV not yet recieved.");
    return true;
  }

  ROS_INFO("[OccupancyGridMerger]: Testing.");
  
  // recieved og
  nav_msgs::OccupancyGridPtr uav_og_p = boost::make_shared<nav_msgs::OccupancyGrid>();
  nav_msgs::OccupancyGridPtr rec_og_p = boost::make_shared<nav_msgs::OccupancyGrid>();
  // copy pose from odometry of current uav
  geometry_msgs::PosePtr odom_pose_p = boost::make_shared<geometry_msgs::Pose>(sh_odometry_.getMsg()->pose.pose);
  geometry_msgs::PosePtr odom_pose_rec_p = boost::make_shared<geometry_msgs::Pose>(sh_odometry_rec1_.getMsg()->pose.pose);

  double cur_yaw = merger2d::getYaw(odom_pose_p->orientation);
  double rec_yaw = merger2d::getYaw(odom_pose_rec_p->orientation);
  if (isnan(cur_yaw) || isnan(rec_yaw)) {
    ROS_WARN("[OccupancyGridMerger]: Pose orientation of one of the UAVs is not a number!!!");
    return true;
  }

  { 
    std::scoped_lock lock(mutex_uav_og_);
    nav_msgs::OccupancyGridConstPtr uav_og_msg = sh_uav_og_.getMsg();
    uav_og_p->data = uav_og_msg->data;
    uav_og_p->header = uav_og_msg->header;
    uav_og_p->info = uav_og_msg->info;
  }

  {
    std::scoped_lock lock(mutex_rec_og_);
    nav_msgs::OccupancyGridConstPtr rec_og_msg = sh_rec_og_.getMsg();
    rec_og_p->data = rec_og_msg->data;
    rec_og_p->header = rec_og_msg->header;
    rec_og_p->info = rec_og_msg->info;
  }

  // --------------------------------------------------------------
  // |                        MERGING BLOCK                       |
  // --------------------------------------------------------------

  nav_msgs::OccupancyGridPtr merged_map;
  {  
      std::scoped_lock lock(mutex_merger_idle_time_);
    EvolutionParams params = {
      .accept_thr = &_acceptance_threshold_, 
      .occup_thr = &_occupancy_threshold_, 
      .pop_size = &_population_size_,
      .gui = &_gui_,
      .min_mic_sec = &_min_micro_sec_,
      .max_mic_sec = &_max_micro_sec_,
      .err_range = &_err_range_,
      .err_angle = &_err_angle_,
      .starting_close = _starting_close_,
      .pub_rel_pose = pub_relative_map_pose_,
			.pub_occup_cells_rec = pub_occupied_cells_rec_,
			.pub_occup_cells_cur = pub_occupied_cells_cur_,
      .pub_consistent_cells = pub_consistent_cells_,
			.pub_gen_poses = pub_generation_poses_,
      .rec_og_frame_id = rec_og_p->header.frame_id,
      .uav_og_frame_id = uav_og_p->header.frame_id,
      .rec_og_stamp = rec_og_p->header.stamp,
      .uav_og_stamp = uav_og_p->header.stamp,
      .rec_og_origin = rec_og_p->info.origin,
      .uav_og_origin = uav_og_p->info.origin,
      .rec_og_resolution = rec_og_p->info.resolution,
      .uav_og_resolution = uav_og_p->info.resolution
    };

    if(!_first_time_merging_){
      _merger_.setEvolutionParams(params);
      _merger_.setMemberVars(uav_og_p, rec_og_p);
      _merger_.initializeEvolver();
      _merger_.setLocalPoses(odom_pose_p, odom_pose_rec_p);
    } else{
      _merger_ = merger2d::Merger2d(uav_og_p, rec_og_p, odom_pose_p, odom_pose_rec_p, params);
      _merger_.initializeEvolver();
      _merger_.initializeGAGeneration();
      _first_time_merging_ = false;
    }
    merged_map = _merger_.mergeMapsContinuously();

    merged_map->header.seq = _merging_info_.seq;  
    ++_merging_info_.seq;
    _merging_info_.previous_rel_pose = boost::make_shared<geometry_msgs::Pose>(*_merger_.getRelativeMapPose());
    _merging_info_.has_rel_pose = true;
    _merging_info_.prev_loc_cur_p = boost::make_shared<geometry_msgs::Pose>(*odom_pose_p);
    _merging_info_.prev_loc_rec_p = boost::make_shared<geometry_msgs::Pose>(*odom_pose_rec_p);
    _merging_info_.previous_acceptance_index = _merger_.getAcceptanceIndex();
  }
  // MERGING BLOCK END

  try {
    pub_og_map_new_uav_.publish(merged_map);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_og_map_new_uav_.getTopic().c_str());
  }

  res.success = true;
  res.message = "Test was successful.";
  
  return true;
}
//}

// | -------------- dynamic reconfigure callback -------------- |

/* //{ callbackDynamicReconfigure() */
void OccupancyGridMerger::callbackDynamicReconfigure([[maybe_unused]] Config& config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_)
    return;

  ROS_INFO(
      "[OccupancyGridMerger]:"
      "Reconfigure Request: "
      "Acceptance threshold: %.2f",
      config.acceptance_threshold);
  ROS_INFO(
      "[OccupancyGridMerger]:"
      "Reconfigure Request: "
      "Occupancy threshold: %.2f",
      config.occupancy_threshold);
  ROS_INFO(
      "[OccupancyGridMerger]:"
      "Reconfigure Request: "
      "Population size: %4d",
      config.population_size);
  ROS_INFO(
      "[OccupancyGridMerger]:"
      "Reconfigure Request: "
      "Minimal time of evolution in micro seconds: %4d",
      config.min_micro_sec);
  ROS_INFO(
      "[OccupancyGridMerger]:"
      "Reconfigure Request: "
      "Maximal time of evolution in micro seconds: %4d",
      config.max_micro_sec);
  ROS_INFO(
      "[OccupancyGridMerger]:"
      "Reconfigure Request: "
      "Range of position initialization in percent: %2f",
      config.err_range);
  ROS_INFO(
      "[OccupancyGridMerger]:"
      "Reconfigure Request: "
      "Range of orientation yaw initialization in percent: %2f",
      config.err_angle);

  {
    std::scoped_lock lock(mutex_merger_idle_time_);

    _acceptance_threshold_ = config.acceptance_threshold;
    _occupancy_threshold_ = config.occupancy_threshold;
    _population_size_ = config.population_size;
    _min_micro_sec_ = config.min_micro_sec;
    _max_micro_sec_ = config.max_micro_sec;
    _err_range_ = config.err_range;
    _err_angle_ = config.err_angle;
  }
}
//}

// | -------------------- support functions ------------------- |

/* copyOccupancyGridInfoAndHeader() //{ */

/**
 * @brief copy info and header of occupancy grid
 *
 * @param dest  destination where info and header are being written to
 * @param src   source occupancy grid of info and header
 */
void OccupancyGridMerger::copyOccupancyGridInfoAndHeader(nav_msgs::OccupancyGridPtr dest, nav_msgs::OccupancyGridConstPtr src){

  // copy header and info
  dest->header = src->header;
  dest->info = src->info;
  // configure header
  dest->header.seq = 0; // update elsewhere
  dest->header.stamp = src->header.stamp; // call ros::Time::now() to refresh stamp
}

//}

}  // namespace occupancy_grid_merger

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(occupancy_grid_merger::OccupancyGridMerger, nodelet::Nodelet);
