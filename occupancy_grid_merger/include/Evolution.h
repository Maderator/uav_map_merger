/**
 * @file Evolution.h
 * @author Jan Maděra <maderja@fel.cvut.cz> (janmadera97@gmail.com)
 * @version 0.0.1
 * @date 2020-06-11
 */

#pragma once
#ifndef EVOLUTION_H
#define EVOLUTION_H

/* INCLUDES //{ */

/* My includes */
#include "Population.h"
#include "Generation.h"
#include "transformations.h"
#include "types.h"
#include "verification.h"

// for visualization of poses and occupied cells
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

// for std::max
#include <algorithm>

// for std::pow
#include <cmath>

// for introduction of noise to pose initialization
#include <random>
// #include <chrono> // used to make more entropy

/* for smart pointers (do not use raw pointers) */
#include <boost/make_shared.hpp> 
#include <boost/shared_ptr.hpp>
#include <memory>

#include <utility>
#include <vector>

/* for storing information about the state of the uav */
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <opencv2/core.hpp>

/* ROS includes for data transformation between ROS and OpenCV */
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h> 

//}

namespace merger2d {

namespace evolution {

class Evolution {

public:

  /* CONSTRUCTORS //{ */  
 
  Evolution();

  Evolution(
      EvolutionParams *params,
      cv_bridge::CvImagePtr cv_cur,
      cv_bridge::CvImagePtr cv_rec,
      OccupancyGridCellsPtr cells_cur,
      OccupancyGridCellsPtr cells_rec,
      geometry_msgs::PosePtr loc_cur_p,
      geometry_msgs::PosePtr glob_cur_p,
      geometry_msgs::PosePtr loc_rec_p,
      geometry_msgs::PosePtr glob_rec_p,
      float occ_thr_img,
      float ori_err_range,
      float pos_err_range
      );

  Evolution(
      EvolutionParams *params,
      cv_bridge::CvImagePtr cv_cur,
      cv_bridge::CvImagePtr cv_rec,
      OccupancyGridCellsPtr cells_cur,
      OccupancyGridCellsPtr cells_rec,
      geometry_msgs::PosePtr loc_cur_p,
      geometry_msgs::PosePtr loc_rec_p,
      float occ_thr_img,
      float ori_err_range,
      float pos_err_range,
      geometry_msgs::PosePtr previous_rel_pose,
      geometry_msgs::PosePtr prev_loc_cur_p,
      geometry_msgs::PosePtr prev_loc_rec_p
      );

  //}
  
  /**
   * @brief Initialize generation for iterative evolution from received poses
   *
   * @param p_size Desired size of initial population
   * @param acceptance_threshold
   *
   * @return Generation prepared for evolution
   */
  GenerationPtr initializeGAGeneration(int p_size, float acceptance_threshold);

  /**
   * @brief Evolve current generation to next generation
   *
   * 1. Compute fitness of each individual pose in the population by 
	 *    method computeMapConsistency()
   * 2. Compute mean likelihood of the population. If an individual is above mean,
   *    assign t to an elite group, otherwise assign it to an inferior group.
   * 3. Mutate the elite group individuals.
   * 4. Replace the inferior group individuals
   *
   * @param generation Population of poses to be evolved by Genetic algorithm
   * @param rel_pose Relative pose of second UAV computed by map merging alg.
   * @param poses_vis array of poses to be visualized
   * @param occup_r_vis array of occupied cells of recieved UAV map to be visualized 
   * @param occup_c_vis array of occupied cells of current UAV map to be visualized 
   * @param consist_c_vis array of cells that are occupied in both maps to be visualized
   */
  void iterate(GenerationPtr generation, geometry_msgs::PoseStampedPtr rel_pose, geometry_msgs::PoseArrayPtr poses_vis, visualization_msgs::MarkerPtr occup_r_vis, visualization_msgs::MarkerPtr occup_c_vis, visualization_msgs::MarkerPtr consist_c_vis);

  /**
   * @brief  Evolve generation until it runs out of time or accepted
   *
   * @param min_ms Minimal duration of evolution in miliseconds.
   * @param max_ms Maximal duration of evolution in miliseconds.
   * @param generation Initial generation to be evolved.
   * @param params Evolution parameters with publishers needed for rviz visualization of poses and transformed occupied cells.
   *
   * @return Last generation  
   */
  void evolve(int min_ms, int max_ms, GenerationPtr generation);

  /**
   * @brief relative pose of the second UAV in the merged map   
   * 
   * @return relative pose of second UAV in the merged map  
   */
  geometry_msgs::PoseConstPtr getRelativePoseOfUAV();

private:  
  
  /* MEMBER VARIABLES //{ */ 

  geometry_msgs::PosePtr rec_uav_pose_; //!< Relative pose relative to the other UAV computed by map merging process.

  EvolutionParams *params_; //!< parameters for evolution

  /// Uniform real distributions for yaw and position generation (with pseudo random generator included)
  RandomDistributions distributions_;

  /// distributions used for mutation of yaw and position in individuals 
  MutationDistributions mutation_distributions_;

  /// Initial prediction of pose
  geometry_msgs::PosePtr initial_pose_;

  /// transformation to align UAV starting locations of UAV in received map with the UAV in current map
  geometry_msgs::PosePtr map_alignment_; 

  /// Map of current uav
  cv_bridge::CvImagePtr cv_cur_;

  /// Recieved map from other uav
  cv_bridge::CvImagePtr cv_rec_;

  /// Local maximum occupied cells
  OccupancyGridCellsPtr occupied_cells_cur_; 
  
  /// Local maximum occupied cells
  OccupancyGridCellsPtr occupied_cells_rec_;

  OccupancyGridCellsPtr consistent_cells_;
  
  /// position of uav inside current map (local current pose)
  geometry_msgs::PosePtr loc_cur_p_;
  
  /// estimated position of uav in the world (global current pose)
  geometry_msgs::PosePtr glob_cur_p_;

  /// position of other uav inside recieved map (local recieved pose)
  geometry_msgs::PosePtr loc_rec_p_;
  
  /// estimated position of other uav in the world (global recieved pose)
  geometry_msgs::PosePtr glob_rec_p_;
 
  /// estimated relative pose of maps from previous merging 
  geometry_msgs::PosePtr previous_rel_pose_;

  /// local received pose used in previous merging
  geometry_msgs::PosePtr prev_loc_cur_p_;

  /// local current pose used in previous merging
  geometry_msgs::PosePtr prev_loc_rec_p_;

  float occ_thr_img_; //!< Occupancy threshold value for image. It stores minimal value of each cell
                      //!< to be taken as occupied (cell value 127 means it is unoccupied, 255
                      //!< occupied... occ_thr_img_ should be somewhere in between).

  
  /// error of rotation around z axis
  float orientation_error_range_;

  /// range on axis with highest error range
  float position_error_range_;

  /// generator for random numbers
  std::mt19937 gen_;

  /// distribution for random choice between genetic operations on inferior group
  std::uniform_int_distribution<> gen_op_dist_;

  std::uniform_real_distribution<> lin_comb_dist_;

  //}
  
  /* INITIALIZATION //{ */
  
  /**
   * @brief Compute initial relative pose by compounding initial pose estimates
   */
  void computeInitialRP();

  /**
   * @brief Compute initial relative pose from previously computed relative pose
   */
  void approximateInitialRP();

  /**
   * @brief Computes map_alignment_ member variable
   */
  void computeMapAlignment();
  
  /**
   * @brief Align the starting UAV positions in maps so that the map merging Population is generated around the starting positions of UAVs.
   */
  void alignUAVMapPositions();

  /**
   * @brief Check if the position of pose is out of boundary given by position_error_range_ and map_alignment_.
   *
   * @param pose Pose to be checked.
   *
   * @return true if the position is out of boundary, false if not
   */
  bool checkPositionOutOfBound(geometry_msgs::PoseConstPtr pose);

  /**
   * @brief Initialies uniform real distribution for initialization of yaw and position of Individual
   *
   * @param gen Initialized generator 
   */
  void initializeDistributions(std::mt19937 gen);
  
  /**
   * @brief Initialies uniform real distribution for mutation of yaw and position of Individual
   *
   * @param gen Initialized generator 
   */
  void initializeMutationDistributions(std::mt19937 gen);

  /**
   * @brief Initialize pose population for evolutionary Genetic algorithm around initial pose
   *
   * @param p_size Desired size of initial population
   *
   * @return Population consisting of poses
   */
  PopulationPtr initializeRPPopulation(int p_size);
 
  /**
   * @brief add some noise to position and orientation of pose bounded by error ranges
   *
   * @param pose Pose for which the method introduces noise
   * @param dist Distributions and generator for noise
   */
  void introduceNoise(geometry_msgs::PosePtr pose, RandomDistributions* dist);
  void introduceNoise(geometry_msgs::PosePtr pose, MutationDistributions* dist);
  
  /**
   * @brief Initialize mt19937 random generator
   *
   * @param random_seed if true,  the seed will be random, if false, the seed will be 100
   */
  std::mt19937 initRandomNumberGenerator(bool random_seed = true);

  /**
   * @brief  This is simple pseudo random seed generator with low entropy.
   *
   * @return "random" seed
   */
  std::mt19937::result_type getRandomSeed();

  //}
  
  /* EVOLUTION //{ */

  /* SUPPORT FUNCTIONS //{ */
 
  /**
   * @brief Compute number of attempts to mutate individual based on its map consistency
   *
   * First it remaps consistency from interval <min_cons,max_cons> to interval <0,1> and
   * then computes number of attempts like this:
   * attempts = std::pow(new_consistency,10)*100;
   * The remapped max_cons will get 100 attempts and all individuals with smaller
   * consistency will get exponentially less attempts. But every individual gets 
   * at least one attempt
   *
   * @param cons Consistency of individual
   * @param min_cons Minimal consistency in population
   * @param max_cons Maximal consistency in population
   *
   * @return number of attempts for given consistency  
   */
  unsigned computeNumberOfAttempts(float cons, float min_cons, float max_cons);

  /**
   * @brief Mutate individuals pose for given number of attempts
   *
   * Change original individual to the attempt with highest map consistency if it has
   * higher map consistency than original individual.
   *
   * @param ind Individual to be mutated
   * @param attempts Number of attempts for better mutation than original individual
   * @param idx Index of individual in population
   * @param population Population in which the individual is
   */
    void mutateIndividual(IndividualPtr ind, unsigned attempts, size_t idx, PopulationPtr population);
  
    /**
     * @brief Create new mutated pose with use of mutation distributions for random noise
     *
     * @param pose Pose from which the new pose is created and mutated
     *
     * @return Mutated pose
     */
    geometry_msgs::PosePtr mutatePose(const geometry_msgs::PoseConstPtr pose);

  //}

  /* MAIN FUNCTIONS //{ */
  
    /**
     * @brief Mutate the elite group of individuals
     *
     * If mutation of individual has higher likelihood then the origianl, replace it 
     * with its mutation; otherwise, keep it unchanged. Better individuals tends
     * to get more mutations.
     */
    void mutateEliteGroup(PopulationPtr elite_group);
  
    /**
     * @brief Replace the inferior group individuals by randomly applying genetic operations.
     *
     * Genetic operations are mutation of pose from elite group, crossover parts, 
     * crossover linear combination, and reinitialization of pose. The best individual
     * from elite group is selected at least once and mutated
     *
     * @param gen Generation in which the inferior group is to be replaced
     */
    void replaceInferiorGroup(GenerationPtr gen);
  
  	/**
     * @brief Mutate individual from elite group
  	 * 
     * Randomly select an individual from the elite group and mutate it.
     *
     * @param sup_ind Individual in superior group to be used as base for 
     * the mutation of individual.
     * @param idx Index of individual in inferior population to be replaced
     * @param inf_pop Inferior population
     */
    void mutateIndividualFromEliteGroup(const IndividualConstPtr sup_ind, size_t idx, PopulationPtr inf_pop);
  
    /**
  	 * @brief Mix position and orientation parts of superior Individuals and replace inferior Individual
     *
     * @param sup_ind1 Individual in superior group to be used in crossover 
     * @param sup_ind2 Individual in superior group to be used in crossover 
     * @param idx Index of individual in inferior population to be replaced
     * @param inf_pop Inferior population
     */
    void crossoverParts(IndividualConstPtr sup_ind1, IndividualConstPtr sup_ind2, size_t idx, PopulationPtr inf_pop);
  
    /**
  	 * @brief Create linear combination of two individuals from superior group
     *
     * @param sup_ind1 Individual in superior group to be used in crossover 
     * @param sup_ind2 Individual in superior group to be used in crossover 
     * @param idx Index of individual in inferior population to be replaced
     * @param inf_pop Inferior population
     */
    void crossoverLinearCombination(IndividualConstPtr sup_ind1, IndividualConstPtr sup_ind2, size_t idx, PopulationPtr inf_pop);
  
    /**
     * @brief Reinitialize individual pose as in first generation of population
     *
     * @param idx Index of individual in inferior population to be reinitialized
     * @param inf_pop Inferior population
     */
    void reinitializeRP(size_t idx, PopulationPtr inf_pop);
  
  //}
  
  //}

    /**
     * @brief Compute pose of the second UAV in the merged map relative to the current UAV
     *
     * see article Multivehicle Cooperative Local Mapping: A Methodology Based on Occupancy Grid Map Merging by Hao Li et al. section V. for more info: https://ieeexplore.ieee.org/document/6776469 article
     *
     * @param map_rel_pose Best relative pose of the two maps being merged
     */
  void computeRelativePoseOfUAV(geometry_msgs::PoseConstPtr map_rel_pose);
    
/* VISUALIZATION methods //{ */

  /**
   * @brief Find cells that are consistent between the two maps
   *
   * @param rel_pose Relative pose of the two maps
   */
    void findConsistentCells(geometry_msgs::PoseConstPtr rel_pose);
  
    /**
     * @brief  Initializes PoseStamped message of relative pose of second UAV
     *
     * @param rel_pose Relative pose of second UAV computed by map merging
     *
     * @return Pose msg to be published  
     */
    geometry_msgs::PoseStampedPtr initRelPose(geometry_msgs::PosePtr rel_pose);

    geometry_msgs::PoseArrayPtr initPoseArray(GenerationPtr gen, float map_res);

    /**
     * @param color 0 - green, 1 - red, 2 - blue
     */
    visualization_msgs::MarkerPtr initMarker(OccupancyGridCellsConstPtr og_cells, geometry_msgs::PoseConstPtr best_pose, float map_resolution, int color);
    visualization_msgs::MarkerPtr initMarker(OccupancyGridCellsConstPtr og_cells, float map_resolution, int color);

    /**
     * @brief publish pose array and markers of occupied cells
     *
     * @param poses PoseArray to be published
     * @param occup_cells_rec Marker message of received occupied cells to be published
     * @param occup_cells_cur Marker message of occupied cells of current UAVs occupancy grid map to be published
     */
    void sendVisualization(geometry_msgs::PoseStampedPtr rel_pose, geometry_msgs::PoseArrayConstPtr poses, visualization_msgs::MarkerConstPtr occup_cells_rec, visualization_msgs::MarkerConstPtr occup_cells_cur, visualization_msgs::MarkerConstPtr consistent_cells);

    void updateRelPose(geometry_msgs::PoseStampedPtr rel_pose_vis, geometry_msgs::PoseConstPtr rel_pose);

    void updatePoseArray(geometry_msgs::PoseArrayPtr p_arr, GenerationPtr gen, float map_res);
    
    /**
     * @brief Updates the occupied cells visualisation by new pose 
     *
     * @param occup_cells Occupied cells to be updated
     * @param map_res Resolution of the occupancy grid map
     * @param new_pose Pose by which the occupied cells are transformed
     */
    void updateOccupiedCells(visualization_msgs::MarkerPtr occup_cells, float map_res, geometry_msgs::PoseConstPtr new_pose);
    
    void updateVisualization(geometry_msgs::PoseStampedPtr rel_pose, geometry_msgs::PoseArrayPtr poses, visualization_msgs::MarkerPtr occup_cells_rec, visualization_msgs::MarkerPtr occup_cells_cur, visualization_msgs::MarkerPtr consist_cells, GenerationPtr gen, float rec_map_resolution, float cur_map_res);

//}

  /* SUPPORT FUNCTIONS //{ */
  
//  /**
//   * NOT NEEDED FOR NOW
//   */
//  EulerAngles quaternionToEulerAngles(geometry_msgs::Quaternion q);
  
  /** 
   * @brief sets orientationErrorRange and positionErrorRange
   * @param orientationErr orientationErrorRange value
   * @param positionErr positionErrorRange value
   */
  void setErrorRanges(float orientation_err, float position_err);
  
  /**
   * @brief Converts current uav's pose to units of meters from cells units
   *
   * @param pose Pose with position given in "cells" units that will be converted to meters
   * @param origin Origin of the occupancy grid map in which the pose is located
   * @param res Resolution of current occuapncy grid map
   */
  void convertPoseUnitsFromResToMeters(geometry_msgs::PosePtr pose, const geometry_msgs::Pose& origin, float res);
  geometry_msgs::PosePtr convertPoseUnitsFromResToMetersNew(const geometry_msgs::PoseConstPtr pose, const geometry_msgs::Pose& origin, float res) const;

  geometry_msgs::PosePtr convertPoseUnitsFromResToMetersWithZeroOriginNew(const geometry_msgs::PoseConstPtr pose, float res) const;

  /**
   * @brief Initialize distributions used in replacement of inferior group
   */
  void initializeReplacementDistributions();

  //}

  /* DEBUG FUNCTIONS //{ */
  
  /**
   * @brief Prints values in CvImage
   *
   * @param img_map Image with data to be printed
   */
  void printValuesInCvImage(cv_bridge::CvImageConstPtr img_map);

  //}
  
};

} // namespace evolution

} // namespace merger2d
 
#endif // EVOLUTION_H
