/**
 * @file
 * @brief declaration of generation of one iteration in evolution
 * @author Jan Maděra <maderja1@fel.cvut.cz> (janmadera97@gmail.com)
 * @date 2020-06-16
 */


#ifndef GENERATION_H
#define GENERATION_H

#include "Individual.h"
#include "Population.h"

#include <memory>
#include <ros/static_assert.h>
#include <stdexcept>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

namespace merger2d {

namespace evolution {

class Population;
class Generation;

typedef std::shared_ptr<Generation> GenerationPtr;
typedef std::shared_ptr<Generation const> GenerationConstPtr;

/**
 * @brief Class with convenient methods for operations on generation in genetic algorithm evolution
 */
class Generation : public Population
{
public:

  Generation() = delete; 
  
  /**
   * @brief Constructor
   * @param population initial population
   * @param meanLikelihood meanLikelihood of given population
   * @param accept_thr Acceptance threshold which indicates how consistent must the two maps be.
   */
  Generation(PopulationPtr population, float accept_thr);

  typedef std::shared_ptr<Generation> Ptr;
  typedef std::shared_ptr<Generation const> ConstPtr;

  /**
   * @return Pose of individual with highest map consistency
   */
  geometry_msgs::PoseConstPtr getBestPose() const;
 
  /**a
   * @return Individual with highest map consistency
   */
  IndividualConstPtr getBestIndividual() const;
  
  /**
   * @return If the best pose has map consistency value higher than acceptance threshold
   */
  bool isAcceptable() const;

  /**
   * @return Acceptance threshold
   */
  float getAcceptanceThreshold() const;

  /**
   * @return Mean likelihood
   */
  float getMeanLikelihood() const;
  
  /**
   * @param new_ml New mean likelihood
   */
  void setMeanLikelihood(float new_ml);

  /**
   * @return Modifiable superior group
   */
  PopulationPtr getSuperiorGroup();
  
  /**
   * @return Modifiable inferior group
   */
  PopulationPtr getInferiorGroup();
 
  /**
   * @brief Divides population into inferior and superior group based on mean likelihood. Mean likelihood is computed at the beginning of this method.
   *
   * @param img_map1 Map of current drone
   * @param img_map2 Recieved map that will be transformed by population of poses
   */
  void dividePopulation();

  /**
   * @brief Compute mean likelihood of population
   *
   * @param img_map1 Map of current drone
   * @param img_map2 Recieved map that will be transformed by population of poses
   *
   * @return Mean likelihood of population.
   */
  float computeMeanLikelihood();

private:

  PopulationPtr inferior_group_; //!< Part of population of poses that has mean consistency lower than mean likelihood

  PopulationPtr superior_group_; //!< Part of population of poses that has mean consistency higher or equal to mean likelihood
  
  IndividualPtr best_individual_; //!< Individual with highest map consistency value

  float mean_likelihood_; //!< Mean of consitency values of individual poses

  float acceptance_threshold_; //!< best_individual_ is accepted only if percent of 
                               //!< occupied and unoccupied cells that overlap out of all
                               //!< overlaps of unoccupied and occupied cells is less
                               //!< than (1 - acceptance_threshold_) * 100. Other cells
                               //!< should overlap only with same type of cell 
                               //!< (occup -> occup, unoccup -> unoccup) or at least one
                               //!< should have unknown value.

  /**
   * @brief If superior group is larger than inferior group then move individuals to inferior group so that each group makes about a half of the generation.
   */
  void equalizeGroups();  

};

} // namespace evolution

} // namespace merger2d

#endif // GENERATION_H
