/**
 * @file
 * @brief declaration of generation of one iteration in evolution
 * @author Jan Maděra <maderja1@fel.cvut.cz> (janmadera97@gmail.com)
 * @date 2020-06-16
 */

#include "Generation.h"

namespace merger2d {

namespace evolution {

// PUBLIC METHODS

  /* CONSTRUCTOR //{ */
  
  Generation::Generation(PopulationPtr population, float accept_thr) :
    Population(population->getPopulation()), acceptance_threshold_(accept_thr) {
      mean_likelihood_ = computeMeanLikelihood();
    }
  
  //}

  /* isAcceptable() method //{ */
  bool Generation::isAcceptable() const {
    if (best_individual_->getMapConsistency() > acceptance_threshold_) {
      return true;
    }
    return false;
  }
  //}
  
  /* GETTERS AND SETTERS //{ */
  
  geometry_msgs::PoseConstPtr Generation::getBestPose() const{
    return best_individual_->getPose();
  }
  
  IndividualConstPtr Generation::getBestIndividual() const{
    return best_individual_;
  }
  
  float Generation::getAcceptanceThreshold() const {
    return acceptance_threshold_;
  }
  
  
  float Generation::getMeanLikelihood() const {
    return mean_likelihood_;
  }
  
  void Generation::setMeanLikelihood(float new_ml) {
    mean_likelihood_ = new_ml;
  }
  
  PopulationPtr Generation::getSuperiorGroup() {
    return superior_group_;
  }
  
  PopulationPtr Generation::getInferiorGroup() {
    return inferior_group_;
  }
  
  //}

  /* dividePopulation() method //{ */
  void Generation::dividePopulation() {
    computeMeanLikelihood();
    std::shared_ptr<std::vector<IndividualPtr>> population = getPopulation();
    float m_cons;
    inferior_group_ = std::make_shared<Population>(); 
    superior_group_ = std::make_shared<Population>();
    for(auto ind : *population) {
      m_cons = ind->getMapConsistency();
      if(m_cons < mean_likelihood_)
        inferior_group_->addIndividual(ind);
      else
        superior_group_->addIndividual(ind);
    }
  }
  //}

  /* computeMeanLikelihood() method //{ */
  float Generation::computeMeanLikelihood() {
    std::shared_ptr<std::vector<IndividualPtr>> population = getPopulation();
    float sum_consistency = 0;
    float best_cons = -1;
    float cur_cons;
    for(auto ind : *population) {
      cur_cons = ind->getMapConsistency();
      if(cur_cons > best_cons) {
        best_cons = cur_cons;
        best_individual_ = ind;
      }
      sum_consistency += cur_cons;
    }
  
    float new_mean_likelihood = sum_consistency / population->size();
    mean_likelihood_ = new_mean_likelihood;
    return new_mean_likelihood;
  }
  //}

// PRIVATE METHODS
 
  /* equalizeGroups() method //{ */
  void Generation::equalizeGroups() {
    // superior group is now larger than inferior group
    size_t sup_sz = superior_group_->getPopulation()->size();
    size_t inf_sz = inferior_group_->getPopulation()->size();
    int mv_num = (sup_sz - inf_sz)/3.0;
  
    bool popped_best_indiv = false;
    IndividualPtr ind;
    for(int i = 0; i < mv_num; ++i) {
      ind = superior_group_->popIndividual(0);
      if(ind == best_individual_){
        popped_best_indiv = true;
        ind = superior_group_->popIndividual(0);
      }
      inferior_group_->addIndividual(ind);
    }
    if(popped_best_indiv){
      superior_group_->addIndividual(best_individual_);
    }
  }
  //}
   
} // namespace evolution

} // namespace merger2d
