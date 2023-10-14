/**
 * @file types.h
 * @brief  
 * @author Jan Maděra <maderja1@fel.cvut.cz> (janmadera97@gmail.com)
 * @date 2020-06-21
 */

#include <vector>
#include <utility>
#include <memory>
#include <random>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>

#ifndef TYPES_H
#define TYPES_H

#define IMG_MAX_OCCUP_VAL 255
#define IMG_ZERO 128
#define IMG_UNKNOWN 0

typedef struct TimerMergingInfo {
  uint32_t seq;
  geometry_msgs::PoseConstPtr previous_rel_pose;
  bool has_rel_pose;
  geometry_msgs::PoseConstPtr prev_loc_cur_p;
  geometry_msgs::PoseConstPtr prev_loc_rec_p;
  float previous_acceptance_index;
} TimerMergingInfo;

  /**
   * @brief Parameters for evolution
   *
   * @param accept_thr Acceptance threshold pointer
   * @param occup_thr Occupancy threshold pointer
   * @param pop_size Population size pointer
   * @param min_mic_sec minimal time in microseconds for which the evolution will be running
   * @param max_mic_sec maximal time in microseconds for which the evolution will be running
   */
typedef struct EvolutionParams {
  float *accept_thr;
  float *occup_thr;
  int *pop_size;
  bool *gui;
  int *min_mic_sec; 
  int *max_mic_sec;
  float *err_range;
  float *err_angle;
  bool starting_close;
  ros::Publisher pub_rel_pose;
  ros::Publisher pub_occup_cells_rec;
  ros::Publisher pub_occup_cells_cur;
  ros::Publisher pub_consistent_cells;
  ros::Publisher pub_gen_poses;
  std::string rec_og_frame_id;
  std::string uav_og_frame_id;
  ros::Time rec_og_stamp;
  ros::Time uav_og_stamp;
  geometry_msgs::Pose rec_og_origin;
  geometry_msgs::Pose uav_og_origin;
  float rec_og_resolution;
  float uav_og_resolution;
} EvolutionParams;

typedef struct RandomDistributions {
  std::uniform_real_distribution<> dis_yaw, dis_dist;
  std::mt19937 gen;
} RandomDistributions;

typedef struct MutationDistributions {
  std::normal_distribution<> dis_yaw, dis_dist;
  std::mt19937 gen;
} MutationDistributions;

typedef struct Cell2D {
  Cell2D(int c, int r) : col(c), row(r) {}
  int col; //!< column of occupancy grid where the cell is located
  int row; //!< row of occupancy grid
} Cell2D;

typedef std::shared_ptr<Cell2D> Cell2DPtr;
typedef std::shared_ptr<Cell2D const> Cell2DConstPtr;

typedef std::vector<Cell2DPtr> OccupancyGridCells;
typedef std::shared_ptr<std::vector<Cell2DPtr>> OccupancyGridCellsPtr;
typedef std::shared_ptr<std::vector<Cell2DPtr> const> OccupancyGridCellsConstPtr;

typedef struct EulerAngles{
  float roll;
  float pitch;
  float yaw;
} EulerAngles;

#endif // TYPES_H
