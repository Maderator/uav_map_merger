/**
 * @file
 * @brief declaration of population in every generation of evolution
 * @author Jan Maděra <maderja1@fel.cvut.cz> (janmadera97@gmail.com)
 * @date 2020-06-11
 */


#ifndef POPULATION_H
#define POPULATION_H

#include "Individual.h"

#include <memory> 
#include <algorithm>
#include <limits>
#include <ros/static_assert.h>
#include <stdexcept>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

namespace merger2d {

namespace evolution {

class Population;

typedef std::shared_ptr<Population> PopulationPtr;
typedef std::shared_ptr<Population const> PopulationConstPtr;

/**
 * @brief Class with convenient methods for operations on population in genetic algorithm evolution
 */
class Population
{
public:

  Population(); 

  /**
   * @brief Constructor
   * @param population initial population
   */
  Population(std::shared_ptr<std::vector<IndividualPtr>> population);

  typedef std::shared_ptr<Population> Ptr;
  typedef std::shared_ptr<Population const> ConstPtr;

  /**
   * @return Population of poses
   */
  std::shared_ptr<const std::vector<IndividualPtr>> getPopulation() const;
  
  /**
   * @return Modifiable population of poses 
   */
  std::shared_ptr<std::vector<IndividualPtr>> getPopulation();

  float getBestConsistency();

  float getWorstConsistency();
  
  /**
   * @brief Fast assignemnt of new population
   *
   * @param new_pop New population to be assigned
   */
  void setPopulation(std::shared_ptr<std::vector<IndividualPtr>> new_pop);
  
  /**
   * @brief Very slow copying of the whole population
   *
   * @param new_pop New population to be copied
   */
  void setPopulation(std::shared_ptr<const std::vector<IndividualPtr>> new_pop);

  void addIndividual(IndividualPtr indiv);

  void exchangeIndividual(IndividualPtr new_indiv, int index);

  void changeIndividualPose(geometry_msgs::PosePtr new_pose, int index);

  void deleteIndividual(int index);

  IndividualPtr popIndividual(int index);

private:

  float best_consistency_; //!< Consistency of the best individual

  float worst_consistency_; //!< Consistency of the worst individual

  std::shared_ptr<std::vector<IndividualPtr>> population_; //!< Population of poses

  /**
   * @brief Finds best and worst consistency in this population
   *
   * If population is empty, the best consistency will be 0 and worst will be max value
   * of float.
   */
  void initializeConsistencies();

  /**
   * @brief Check if given consistency is better or worse than current best and worst consistencies
   *
   * @param cons new or updated consistency of current population
   */
  void updateConsistencies(float cons);

  /**
   * @brief Check if individual with worst or best consistency was deleted. If so this will update or reinitialize consistencies
   *
   * @param new_cons Consistency of new individual
   * @param old_cons Consistency of old individual
   * @param new_indiv New individual
   */
  void preserveConsistency(float new_cons, float old_cons);

  /**
   * @brief Check if individual with worst or best consistency was deleted. If so this will reinitialize consistencies
   *
   * @param old_cons Consistency of deleted individual
   */
  void preserveConsistency(float old_cons); 

};

} // namespace evolution

} // namespace merger2d

#endif // POPULATION_H
