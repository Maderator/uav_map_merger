/**
 * @file
 * @brief declaration of an individual in the population of poses
 * @author Jan Maděra <maderja1@fel.cvut.cz> (janmadera97@gmail.com)
 * @date 2020-06-16
 */


#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H

// transformations and conversions
#include "transformations.h"
#include "types.h"

#include <memory>
#include <ros/static_assert.h>
#include <stdexcept>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

namespace merger2d {

namespace evolution {
  
  class Individual;
  
  typedef std::shared_ptr<Individual> IndividualPtr;
  typedef std::shared_ptr<Individual const> IndividualConstPtr;
  

  /**
   * @Brief Population consists of individuals with pose and map consistency
   */
  class Individual
  {
  public:
    Individual() = delete;
  
    /**
     * @brief Takes pose and computes map_consistency of the two maps compounded with given pose
     *
     * @param pose Initial pose of individual
     * @param cells_rec Occupied cells in the map from another drone
     * @param occupancy_threshold Used to select grid cells that tend to be closer to an object (higher the value of cell, higher the probability of cell being occupied by object).
     * @param cur_map_width Width of current drones occupancy grid map
     * @param cur_map_height Height of occup. grid map
     * @param img_map_cur Occupancy grid map of current drone in form of CvImage
     */
    Individual(geometry_msgs::PosePtr pose, OccupancyGridCellsPtr cells_rec, float occupancy_threshold, unsigned cur_map_width, unsigned cur_map_height, cv_bridge::CvImageConstPtr img_map_cur);
    
    /**
     * @return returns consistency of maps when second one is transformed by this pose
     */
    float getMapConsistency() const;
  
    /**
     * @param pose Individuals initial pose
     */
    void setPose(geometry_msgs::PosePtr pose);
  
    /**
     * @param pose Individuals initial pose
     */
    void setPose(geometry_msgs::PoseConstPtr pose);

    /**
     * @param pose Individuals initial pose
     * @param consistency Map consistency of given pose
     */
    void setPose(geometry_msgs::PosePtr pose, float consistency);
  
    /**
     * @return pose of individual
     */
    geometry_msgs::PoseConstPtr getPose() const;
 
  /* OBJECTIVE FUNCTION //{ */
 
  /**
   * @brief Objective function measures consistency degree of two maps where one map is transformed
   *
   * Consistency is computed according to [the paper](https://ieeexplore.ieee.org/document/6776469) like this:
   *  1. Get all points cB in mapB that has maximum occupancy value (the cell is occupied for sure)
   *  2. Transform the points cB by poseAB to get cB'
   *  3. set sum s=0
   *  4. For every transformed point ci in cB' do: 
   *    - If point pA in mapA on which the ci ended after transformation has higher occupancy value than max. value * threshold, then add the value of pA to the sum s.
   *    - Else don't add anything
   *
	 * @return Map consistency value
	 */
  float computeMapConsistency() const;
  
  /**
   * @brief Same as above but with different pose than pose of this individual.
   *
   * @param pose Pose with which this method computes consistency.
   *
   * @return Map consistency of individuals maps with given pose.
   */
  float computeMapConsistencyForGivenPose(geometry_msgs::PosePtr pose) const;
    /**
     * @brief Find out if points x and y coords are inside maps boundaries.
     *
     * @param p Point that is being checked.
     * @param width Width of map.
     * @param height Height of map.
     *
     * @return true if the point is inside boundaries, false if it is not.
     */
    bool isPointInsideMap(geometry_msgs::PointConstPtr p, unsigned width, unsigned height) const;
  //}

  //}

  private:
  
    geometry_msgs::PosePtr pose_; //!< Individuals pose
    
    OccupancyGridCellsPtr cells_rec_; //!<Occupied cells in the map from another drone
  
    float map_consistency_; //!< Consistency of two maps when received map is compounded with individuals pose

  
    float occupancy_threshold_; //!< Used to select grid cells that tend to be closer to an object.
                                //!< Higher the value of cell is (max 255), higher the probability  
                                //!< of cell being occupied by object.
    unsigned cur_map_width_; //!< width of occupancy grid of current drone
    unsigned cur_map_height_; //!< height of occupancy grid of current drone

    /// Occupancy grid of this uav in form of CvImage 
    cv_bridge::CvImageConstPtr img_map_cur_;

  };
  
} // namespace evolution

} // namespace merger2d

#endif // INDIVIDUAL_H
