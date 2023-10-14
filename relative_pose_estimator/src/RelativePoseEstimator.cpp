/* include header file of this class */
#include <RelativePoseEstimator.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace relative_pose_estimator
{

/* onInit() //{ */

void RelativePoseEstimator::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here

  // add booleans here

  /* obtain node handle */
  ros::NodeHandle nh("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader param_loader(nh, "RelativePoseEstimator");

  param_loader.loadParam("rate/publish_relative_pose", _rate_timer_publish_relative_pose_);

  ROS_INFO_STREAM_ONCE("[RelativePoseEstimator]: " << _rate_timer_publish_relative_pose_ << " publish rate of computing relative pose in Hz.");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[RelativePoseEstimator]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ------------------ initialize subscribers ----------------- |
  
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "RelativePoseEstimator";
  shopts.no_message_timeout = ros::Duration(5.0);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_rec_occup_map_ = mrs_lib::SubscribeHandler<nav_msgs::OccupancyGrid>(shopts, "rec_map1_in");
  sh_cur_occup_map_ = mrs_lib::SubscribeHandler<nav_msgs::OccupancyGrid>(shopts, "occupancy_grid_map_in");
  sh_rel_map_pose_             = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, "rel_map_pose_in");
  sh_odometry_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_uav_in");
  sh_odometry_rec1_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_rec1_in");
  /* subscribe ground truth only in simulation, where it is available */
  if (_simulation_) {
    sh_ground_truth_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_gt_in");
  }

  // | ------------------ initialize publishers ----------------- |
  
  pub_relative_pose_ = nh.advertise<geometry_msgs::PoseStamped>("relative_pose_out", 1);
  pub_relative_pose_in_glob_map_ = nh.advertise<geometry_msgs::PoseStamped>("relative_pose_glob_map_out", 1);


  // | -------------------- initialize timers ------------------- |

  timer_publish_relative_pose_    = nh.createTimer(ros::Rate(_rate_timer_publish_relative_pose_), &RelativePoseEstimator::callbackTimerPublishRelativePose, this);

  ROS_INFO_ONCE("[RelativePoseEstimator]: initialized");

  is_initialized_ = true;
}
//}

// | --------------------- timer callbacks -------------------- |

/*  callbackTimerPublishRelativePose() //{ */
void RelativePoseEstimator::callbackTimerPublishRelativePose([[maybe_unused]] const ros::TimerEvent& te){
  
  /* CHECKS, WARNINGS //{ */
  if (!is_initialized_) {
    return;
  }
  
  if(!sh_rel_map_pose_.hasMsg()){
    ROS_WARN_THROTTLE(2.0, "[RelativePoseEstimator]: Relative map pose not yet received from OccupancyGridMerger.");
    return;
  }
  
  if(!sh_cur_occup_map_.hasMsg()){
    ROS_WARN_THROTTLE(2.0, "[RelativePoseEstimator]: UAV map not yet published.");
    return;
  }
  
  if(!sh_rec_occup_map_.hasMsg()){
    ROS_WARN_THROTTLE(2.0, "[RelativePoseEstimator]: Other UAV map not yet received.");
    return;
  }
  
  if(!sh_odometry_rec1_.hasMsg()){
    ROS_WARN_THROTTLE(1.0, "[RelativePoseEstimator]: Cannot compute relative pose of UAVs, odometry from other UAV not yet recieved.");
    return;
  }

  if(!sh_odometry_.hasMsg()){
    ROS_WARN_THROTTLE(1.0, "[RelativePoseEstimator]: Cannot compute relative pose of UAVs, odometry of current UAV not yet recieved.");
    return;
  }
  //}

  /* Set poseStamped //{ */
  
  if(!_pub_rel_pose_first_time_){
    ++_glob_rec_pose_.header.seq;
    ++_rel_rec_pose_.header.seq;
  } else {
    _glob_rec_pose_.header.seq = 0;
    _glob_rec_pose_.header.frame_id = sh_odometry_.getMsg()->header.frame_id;
    _rel_rec_pose_.header.seq = 0;
    _rel_rec_pose_.header.frame_id = sh_odometry_.getMsg()->header.frame_id;
  }
   
  _glob_rec_pose_.header.stamp = ros::Time().now();
  _rel_rec_pose_.header.stamp = ros::Time().now();
  
  //}

  /* global pose publishing //{ */
  geometry_msgs::PoseConstPtr rel_map_pose = boost::make_shared<geometry_msgs::Pose>(sh_rel_map_pose_.getMsg()->pose);
  geometry_msgs::PosePtr rec_loc_p = boost::make_shared<geometry_msgs::Pose>(sh_odometry_rec1_.getMsg()->pose.pose);
  geometry_msgs::PosePtr originDifference = compoundPose(invertPose(sh_rec_occup_map_.getMsg()->info.origin), sh_cur_occup_map_.getMsg()->info.origin);
  _glob_rec_pose_.pose = *compoundPose(originDifference, poseUAVLocalToGlobal(rel_map_pose, rec_loc_p));
  _glob_rec_pose_.pose.position.z = rec_loc_p->position.z;
  
  try {
    pub_relative_pose_in_glob_map_.publish(_glob_rec_pose_);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_relative_pose_in_glob_map_.getTopic().c_str());
  }
  //}

  /* UAVtoUAV pose publishing //{ */
  geometry_msgs::PosePtr cur_loc_p = boost::make_shared<geometry_msgs::Pose>(sh_odometry_.getMsg()->pose.pose);
  _rel_rec_pose_.pose = *recUAVRelToCurUAV(rel_map_pose, cur_loc_p, rec_loc_p);
  
  try {
    pub_relative_pose_.publish(_rel_rec_pose_);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_relative_pose_.getTopic().c_str());
  }
  //}
  return;
}

//}

/* callbackTimerCheckSubscribers() //{ */

void RelativePoseEstimator::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  if (!sh_odometry_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "Not received uav odom msg since node launch.");
  }
  
  if (!sh_odometry_rec1_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "Not received rec uav odom msg since node launch.");
  }

  if (_simulation_) {
    if (!sh_ground_truth_.hasMsg()) {
      ROS_WARN_THROTTLE(10.0, "Not received ground truth odom msg since node launch.");
    }
  }
}
//}

// | -------------------- service callbacks ------------------- |

}  // namespace realtive_pose_estimator

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(relative_pose_estimator::RelativePoseEstimator, nodelet::Nodelet);
