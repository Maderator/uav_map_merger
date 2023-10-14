/**  @file
 *   @author Jan Maděra <maderja\fel.cvut.cz> (janmadera97\gmail.com)
 *   @date 2020-06-11
 *   @copyright The 3-Clause BSD License
 */

#include "Population.h"

namespace merger2d {

namespace evolution {
 
  Population::Population() {
    population_ = std::make_shared<std::vector<IndividualPtr>>();
    initializeConsistencies();
  }
  
  Population::Population(std::shared_ptr<std::vector<IndividualPtr>> population) :
    population_(population) {
    initializeConsistencies();
    }
 

  std::shared_ptr<const std::vector<IndividualPtr>> 
    Population::getPopulation() const 
  { 
    return population_;
  }

  std::shared_ptr<std::vector<IndividualPtr>> Population::getPopulation() { 
    return population_;
  } 

  float Population::getBestConsistency() { return best_consistency_; }

  float Population::getWorstConsistency() { return worst_consistency_; }
  
  void Population::setPopulation(std::shared_ptr<std::vector<IndividualPtr>> new_pop){
    population_ = new_pop;
    initializeConsistencies();
  }

  void Population::setPopulation(std::shared_ptr<const std::vector<IndividualPtr>> new_pop){
    *population_ = *new_pop;
    initializeConsistencies();
  }
 
  void Population::addIndividual(IndividualPtr indiv) {
    population_->push_back(indiv);
    float cons = indiv->getMapConsistency();
    updateConsistencies(cons);
  }

  void Population::exchangeIndividual(IndividualPtr new_indiv, int index) {
    IndividualPtr old_indiv = population_->at(index);
    float old_cons = old_indiv->getMapConsistency();
    float new_cons = new_indiv->getMapConsistency();
    population_->at(index) = new_indiv;
    preserveConsistency(old_cons, new_cons);
  }

  void Population::changeIndividualPose(geometry_msgs::PosePtr new_pose, int index) {
    IndividualPtr indiv = population_->at(index);
    float old_cons = indiv->getMapConsistency();
    indiv->setPose(new_pose);
    float new_cons = indiv->getMapConsistency();
    preserveConsistency(old_cons, new_cons);
  }

  void Population::deleteIndividual(int index) {
    IndividualPtr del_ind = population_->at(index);
    float old_cons = del_ind->getMapConsistency();   
    population_->erase(population_->begin() + index);
    preserveConsistency(old_cons);
  }

  IndividualPtr Population::popIndividual(int index) {
    IndividualPtr ind = population_->at(index);
    float cons = ind->getMapConsistency();   
    population_->erase(population_->begin() + index);
    preserveConsistency(cons);
    return ind;
  }

// private methods

  /* initializeConsistencies() method //{ */
  void Population::initializeConsistencies() {
    best_consistency_ = 0;
    worst_consistency_ = std::numeric_limits<float>::max();
    for(auto ind : *population_){
      float con = ind->getMapConsistency();
      if(con > best_consistency_)
        best_consistency_ = con;
      if (con < worst_consistency_)
        worst_consistency_ = con;
    }
  }
  //}
  
  /* updateConsistencies() method //{ */
  void Population::updateConsistencies(float cons) {
    if(cons < worst_consistency_)
      worst_consistency_ = cons;
    if(cons > best_consistency_)
      best_consistency_ = cons;
  }
  //}
 
  /* preserveConsistency() method //{ */
  void Population::preserveConsistency(float new_cons, float old_cons) { 
    bool best_deleted = (old_cons == best_consistency_ && new_cons < old_cons);
    bool worst_deleted = (old_cons == worst_consistency_ && new_cons > old_cons);
    // if worst or best was deleted without substitute -> reinitialize
    if(best_deleted || worst_deleted) 
      initializeConsistencies();
    // if that is not case just check if given consistency is not the best or worst
    else
      updateConsistencies(new_cons);
  }
    
  void Population::preserveConsistency(float old_cons) { 
    bool best_deleted = (old_cons == best_consistency_);
    bool worst_deleted = (old_cons == worst_consistency_);
    // if worst or best was deleted without substitute -> reinitialize
    if(best_deleted || worst_deleted) 
      initializeConsistencies();
  }
  //}
  

} // namespace evolution

} // namespace merger2d
