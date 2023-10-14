/**  @file
 *   @author Jan Maděra <maderja@fel.cvut.cz> (janmadera97@gmail.com)
 *   @date 2020
 *   @copyright The 3-Clause BDS License
 */

#include "Evolution.h"

namespace merger2d{

namespace evolution {

  /* CONSTRUCTORS //{ */  
 
  Evolution::Evolution(){}

  Evolution::Evolution(
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
      ) : params_(params), cv_cur_(cv_cur), cv_rec_(cv_rec), occupied_cells_cur_(cells_cur), occupied_cells_rec_(cells_rec), loc_cur_p_(loc_cur_p), 
  glob_cur_p_(glob_cur_p), loc_rec_p_(loc_rec_p), glob_rec_p_(glob_rec_p),
  occ_thr_img_(occ_thr_img), orientation_error_range_(ori_err_range),
  position_error_range_(pos_err_range) {

    computeMapAlignment();
    computeInitialRP();
    computeRelativePoseOfUAV(initial_pose_);

    std::mt19937 gen_ = initRandomNumberGenerator(); // use with false argument for deterministic generator
    initializeDistributions(gen_);
    initializeMutationDistributions(gen_);
    initializeReplacementDistributions();
  }

  Evolution::Evolution(
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
      ) : params_(params), cv_cur_(cv_cur), cv_rec_(cv_rec), occupied_cells_cur_(cells_cur), occupied_cells_rec_(cells_rec), loc_cur_p_(loc_cur_p), 
  loc_rec_p_(loc_rec_p), previous_rel_pose_(previous_rel_pose), prev_loc_cur_p_(prev_loc_cur_p), prev_loc_rec_p_(prev_loc_rec_p), occ_thr_img_(occ_thr_img), orientation_error_range_(ori_err_range),
  position_error_range_(pos_err_range) {

    computeMapAlignment();
    approximateInitialRP();
    computeRelativePoseOfUAV(initial_pose_);

    std::mt19937 gen_ = initRandomNumberGenerator(); // use with false argument for deterministic generator
    initializeDistributions(gen_);
    initializeMutationDistributions(gen_);
    initializeReplacementDistributions();
  }
  //}

  /* initializeMutationDistributions() method //{ */
  void Evolution::initializeMutationDistributions(std::mt19937 gen) {
    // init distributions
    std::normal_distribution<> 
      dis_yaw(0, 0.07); // approximately +-4 degrees
    std::normal_distribution<> 
      dis_dist(0, 20); // error of 45 tiles
    mutation_distributions_ = {.dis_yaw = dis_yaw, .dis_dist = dis_dist, .gen = gen};
  }
  //}

  /* initializeDistributions() method //{ */
  void Evolution::initializeDistributions(std::mt19937 gen) {
    // init distributions
    std::uniform_real_distribution<> 
      dis_yaw(-orientation_error_range_, orientation_error_range_);
    std::uniform_real_distribution<> 
      dis_dist(-position_error_range_,position_error_range_);
    distributions_ = {.dis_yaw = dis_yaw, .dis_dist = dis_dist, .gen = gen};
  }
  //}

// Public methods
  
  /* initializeGAGeneration() method //{ */
  GenerationPtr Evolution::initializeGAGeneration(int p_size, float acceptance_threshold) {
    PopulationPtr p = initializeRPPopulation(p_size);
    acceptance_threshold *= occupied_cells_cur_->size(); // Do not use acceptance threshold as a termination condition of the evolution process even thought it is used in the genetic algorithm article. We implemented it in Generation class (not used it) but it is not a robust indicator of the successful map merging.
    return std::make_shared<Generation>(p, acceptance_threshold);
  }
  //}
  
  /* iterate() method //{ */
  void Evolution::iterate(GenerationPtr generation, geometry_msgs::PoseStampedPtr rel_pose, geometry_msgs::PoseArrayPtr poses_vis, visualization_msgs::MarkerPtr occup_r_vis, visualization_msgs::MarkerPtr occup_c_vis, visualization_msgs::MarkerPtr consist_c_vis){
    // Iteration
    generation->dividePopulation();
    mutateEliteGroup(generation->getSuperiorGroup());
    replaceInferiorGroup(generation);
    generation->computeMeanLikelihood();
    
    // Visualization topics computations
    computeRelativePoseOfUAV(generation->getBestPose());
    findConsistentCells(generation->getBestPose());
    updateVisualization(rel_pose, poses_vis, occup_r_vis, occup_c_vis, consist_c_vis, generation, params_->rec_og_resolution, params_->uav_og_resolution); // update visualizations
    sendVisualization(rel_pose, poses_vis, occup_r_vis, occup_c_vis, consist_c_vis); // visualize
  }
  //}

  /* evolve() method //{ */
  void Evolution::evolve(int min_ms, int max_ms, GenerationPtr generation) {
    // create PoseStamped, PoseArray, and Marker for visualization
    geometry_msgs::PoseStampedPtr rel_pose = initRelPose(rec_uav_pose_);
    geometry_msgs::PoseArrayPtr gen_poses_vis = initPoseArray(generation, params_->rec_og_resolution);
    visualization_msgs::MarkerPtr occup_cells_rec_vis = initMarker(occupied_cells_rec_, generation->getBestPose(), params_->rec_og_resolution, 0);
    visualization_msgs::MarkerPtr occup_cells_cur_vis = initMarker(occupied_cells_cur_, params_->uav_og_resolution, 2);
    findConsistentCells(generation->getBestPose());
    visualization_msgs::MarkerPtr consistent_cells = initMarker(consistent_cells_, params_->uav_og_resolution,1);

    ros::Duration cur_time(0);

    int32_t min_s = min_ms/1000000; 
    int32_t min_ns = (min_ms%1000000) * 1000;
    ros::Duration min_time(min_s, min_ns);
    
    int32_t max_s = max_ms/1000000;
    int32_t max_ns = (max_ms%1000000) * 1000;
    ros::Duration max_time(max_s, max_ns);
    
    ros::Time begin = ros::Time::now();

    std::vector<double> cum_time;
    std::vector<float> fitness_func;

    // iterate for at least min_time
    do { 
      iterate(generation, rel_pose, gen_poses_vis, occup_cells_rec_vis, occup_cells_cur_vis, consistent_cells);
      cur_time = ros::Time::now() - begin;
      cum_time.push_back(cur_time.toSec());
      fitness_func.push_back(generation->getBestIndividual()->getMapConsistency());
    } while(cur_time < min_time);
    // iterate until time runs out
    while(cur_time < max_time){
      iterate(generation, rel_pose, gen_poses_vis, occup_cells_rec_vis, occup_cells_cur_vis, consistent_cells);
      cur_time = ros::Time::now() - begin;
      cum_time.push_back(cur_time.toSec());
      fitness_func.push_back(generation->getBestIndividual()->getMapConsistency());
    }
  }
  //}

  /* getRelativePoseOfUAVs() method //{ */
  geometry_msgs::PoseConstPtr Evolution::getRelativePoseOfUAV() {
    return rec_uav_pose_;
  }
  //}

// Protected methods

  /* INITIALIZATION //{ */
  
  // This will be always zero if only loc_cur_p_ is known
  /* computeInitialRP() method //{ */
  void Evolution::computeInitialRP()
  {
    geometry_msgs::PosePtr inv_localB =   invertPose(loc_rec_p_); 
    geometry_msgs::PosePtr inv_globalA =  invertPose(glob_cur_p_); 
    geometry_msgs::PosePtr b =            compoundPose(glob_rec_p_, inv_localB);
    geometry_msgs::PosePtr b_in_globalA = compoundPose(inv_globalA, b);
    initial_pose_ =       compoundPose(loc_cur_p_, b_in_globalA);
    double yaw = getYaw(initial_pose_->orientation);
    setRPY(0, 0, yaw, initial_pose_->orientation);
    if(params_->starting_close){
      alignUAVMapPositions();
    }
  }
  //}
  
  /* approximateInitialRP() method //{ */
  void Evolution::approximateInitialRP() {
    // if the UAVs start close together ->
    // use relative pose from previous map merging
    if(params_->starting_close){
      if(checkPositionOutOfBound(previous_rel_pose_)){ // If the position is out of boundary
        initial_pose_ = previous_rel_pose_;
      } else{
        initial_pose_ = map_alignment_;
      }
    // if the UAVs do NOT start close together ->
    // use relative pose from previous map merging only if the relative pose is not out of boundaries of possible poses
    } else{
      int max_cols = cv_cur_->image.cols > cv_rec_->image.cols ? cv_cur_->image.cols : cv_rec_->image.cols;
      int max_rows = cv_cur_->image.rows > cv_rec_->image.rows ? cv_cur_->image.rows : cv_rec_->image.rows;
      bool x_out_of_bound = abs(initial_pose_->position.x) > max_cols;
      bool y_out_of_bound = abs(initial_pose_->position.y) > max_rows;
      if(x_out_of_bound || y_out_of_bound){
        computeInitialRP();
      } else{
        initial_pose_ = previous_rel_pose_;
      }
    }
  }
  //}
  
  /* computeMapAlignment() method //{ */
  void Evolution::computeMapAlignment() {
    geometry_msgs::Pose* cur_orig = &(params_->uav_og_origin);
    geometry_msgs::Pose* rec_orig = &(params_->rec_og_origin);
    float cur_res = params_->uav_og_resolution;
    float rec_res = params_->rec_og_resolution;
    // received_origin - current_origin
    // initial relative pose translation to align UAV starting positions
    map_alignment_ = boost::make_shared<geometry_msgs::Pose>();
    map_alignment_->position.x = rec_orig->position.x / rec_res - cur_orig->position.x / cur_res; 
    map_alignment_->position.y = rec_orig->position.y / rec_res - cur_orig->position.y /cur_res; 
    map_alignment_->position.z = rec_orig->position.z / rec_res - cur_orig->position.z / cur_res;
    map_alignment_->orientation.x = 0;
    map_alignment_->orientation.y = 0;
    map_alignment_->orientation.z = 0;
    map_alignment_->orientation.w = 1;
    double rec_yaw = getYaw(rec_orig->orientation);
    double cur_yaw = getYaw(cur_orig->orientation);
    setYaw(rec_yaw-cur_yaw,  map_alignment_->orientation);
  }
  //}

  /* alignUAVMapPositions() method //{ */
  void Evolution::alignUAVMapPositions() {
    // align initial_pose
    initial_pose_->position.x += map_alignment_->position.x; 
    initial_pose_->position.y += map_alignment_->position.y;
    double init_yaw = getYaw(initial_pose_->orientation);
    double alig_yaw = getYaw(map_alignment_->orientation);
    setYaw(init_yaw + alig_yaw,initial_pose_->orientation);
  }
  //}

  /* checkPositionOutOfBound() method //{ */
  bool Evolution::checkPositionOutOfBound(geometry_msgs::PoseConstPtr pose) {
    // is in boundary?
    bool x = abs(pose->position.x - map_alignment_->position.x) < position_error_range_;
    bool y = abs(pose->position.y - map_alignment_->position.y) < position_error_range_;
    return x && y;
  }
  //}
 
  /* initializeRPPopulation() method //{ */
  PopulationPtr Evolution::initializeRPPopulation(int p_size){
    PopulationPtr p = std::make_shared<Population>();
    std::shared_ptr<std::vector<IndividualPtr>> individuals = p->getPopulation();
    double yaw = getYaw(initial_pose_->orientation);
    setRPY(0, 0, yaw, initial_pose_->orientation);

    for(int i = 0; i < p_size; ++i){      
      auto pose = boost::make_shared<geometry_msgs::Pose>(*initial_pose_);
      // add random noise to the initial pose in error ranges
      introduceNoise(pose, &distributions_);
      IndividualPtr ind = std::make_shared<Individual>(pose, occupied_cells_rec_, occ_thr_img_, cv_cur_->image.cols, cv_cur_->image.rows, cv_cur_);
      individuals->push_back(ind);
    }
    return p;
  }
  //}

  /* initRandomNumberGenerator() method //{ */
  std::mt19937 Evolution::initRandomNumberGenerator(bool random_seed /* =true */ ){
    if(random_seed)
      return std::mt19937(getRandomSeed());
    else
      return std::mt19937(100);
  }
  //}

  /* getRandomSeed() method //{ */
  std::mt19937::result_type Evolution::getRandomSeed(){
    std::random_device rd;
    std::mt19937::result_type seed = rd();
    return seed;
  }
  //}
 
  /* introduceNoise() method //{ */
  void Evolution::introduceNoise(geometry_msgs::PosePtr pose, RandomDistributions* dist){
    double yaw_noise = dist->dis_yaw(dist->gen);
    double x_pos_noise = dist->dis_dist(dist->gen);
    double y_pos_noise = dist->dis_dist(dist->gen);
    pose->position.x += x_pos_noise;
    pose->position.y += y_pos_noise;
    double yaw = getYaw(pose->orientation);
    yaw += yaw_noise;
    setYaw(yaw, pose->orientation);
  }
  
  void Evolution::introduceNoise(geometry_msgs::PosePtr pose, MutationDistributions* dist){
    double yaw_noise = dist->dis_yaw(dist->gen);
    double x_pos_noise = dist->dis_dist(dist->gen);
    double y_pos_noise = dist->dis_dist(dist->gen);
    pose->position.x += x_pos_noise;
    pose->position.y += y_pos_noise;
    double yaw = getYaw(pose->orientation);
    yaw += yaw_noise;
    setYaw(yaw, pose->orientation);
  }
  //}

  //}

  /* EVOLUTION //{ */

  /* SUPPORT METHODS //{ */
  
  /* computeNumberOfAttempts() method //{ */
  unsigned Evolution::computeNumberOfAttempts(float cons, float min_cons, float max_cons) {         
    float new_cons = remapVarToUnitInterval(cons, min_cons, max_cons);
    return std::max(static_cast<unsigned>(std::pow(new_cons,10) * 100), 1u);
  }
  //}

  /* mutateIndividual() method //{ */
  void Evolution::mutateIndividual(IndividualPtr ind, unsigned attempts, size_t idx, PopulationPtr population) {
    geometry_msgs::PosePtr mut_p = 
      boost::make_shared<geometry_msgs::Pose>(*(ind->getPose()));
    
    for(unsigned i = 0; i < attempts; ++i){
      geometry_msgs::PosePtr temp_p =  mutatePose(mut_p);
      float new_cons = ind->computeMapConsistencyForGivenPose(temp_p);
      if(new_cons > ind->getMapConsistency())
        population->changeIndividualPose(temp_p, idx);
    }
  }
  //}
  
  /* mutatePose() method //{ */
  geometry_msgs::PosePtr Evolution::mutatePose(const geometry_msgs::PoseConstPtr pose) {
    geometry_msgs::PosePtr new_pose = boost::make_shared<geometry_msgs::Pose>(*pose);
    introduceNoise(new_pose, &mutation_distributions_);
    return new_pose;
  }
  //}

  /* initializeReplacementDistributions() method //{ */
  void Evolution::initializeReplacementDistributions(){
    gen_op_dist_ = std::uniform_int_distribution<>(0, 3);
    lin_comb_dist_ = std::uniform_real_distribution<>(0,1);
  }
  //}

  //}

  /* mutateEliteGroup() method //{ */
  void Evolution::mutateEliteGroup(PopulationPtr elite_group){
    auto elite_pop = elite_group->getPopulation();
    float max_cons = elite_group->getBestConsistency();
    //float min_cons = elite_group->getWorstConsistency();
    bool first_max_cons_ind = true;
    size_t p_size = elite_pop->size();
    for(size_t idx = 0; idx < p_size; ++idx) {
      auto ind = elite_pop->at(idx);
      // MORE PERFORMANCE HUNGRY APPROACH
      // unsigned attempts = computeNumberOfAttempts(ind->getMapConsistency(), min_cons, max_cons);
      // mutateIndividual(ind, attempts, idx, elite_group);
      
      // STABLE APPROACH
      if(first_max_cons_ind && ind->getMapConsistency() == max_cons) {
         mutateIndividual(ind, 100, idx, elite_group);
         first_max_cons_ind = false;
       } else {
         mutateIndividual(ind, 1, idx, elite_group);
       }
    }
  }
  //}
  
  /* replaceInferiorGroup() method //{ */
  void Evolution::replaceInferiorGroup(GenerationPtr gen){
    // if there is no individual in inferior group (mean_likelihood_ == 0) return
    if(gen->getInferiorGroup()->getPopulation()->size() == 0)       
      return;
    // get populations
    PopulationPtr inf_pop = gen->getInferiorGroup();
    std::shared_ptr<std::vector<IndividualPtr>> inf_pop_vec = inf_pop->getPopulation();
    std::shared_ptr<std::vector<IndividualPtr>> sup_pop_vec = 
      gen->getSuperiorGroup()->getPopulation();

    // create distribution for selection of generation operation on individual
    std::uniform_int_distribution<>
      sup_pop_dist(0, sup_pop_vec->size()-1);
    
    // mutate best individual and replace ind. in inferior group with it
    
    int gen_op; // chosen generation operation on individual

    size_t inf_p_size = inf_pop_vec->size();
    // cycle that replace the inferior group
    for(size_t idx = 0; idx < inf_p_size; ++idx){
      gen_op = gen_op_dist_(gen_);  
      switch (gen_op) {
      case 0: {
          IndividualPtr sup_ind = sup_pop_vec->at(sup_pop_dist(gen_));
          mutateIndividualFromEliteGroup(sup_ind, idx, inf_pop); 
          break;
        }
      case 1: {
          IndividualPtr sup_ind1 = sup_pop_vec->at(sup_pop_dist(gen_));
          IndividualPtr sup_ind2 = sup_pop_vec->at(sup_pop_dist(gen_));
          crossoverParts(sup_ind1, sup_ind2, idx, inf_pop);
          break;
        }
      case 2: {
          IndividualPtr sup_ind1 = sup_pop_vec->at(sup_pop_dist(gen_));
          IndividualPtr sup_ind2 = sup_pop_vec->at(sup_pop_dist(gen_));
          crossoverLinearCombination(sup_ind1, sup_ind2, idx, inf_pop);
          break;
        }
      // The reinitialization is used in all remaining number of cases If for example the switch distribution is set to generate uniform number in [0,6] interval the reinitialization will be used in roughly 66% of cases
      default: {
          reinitializeRP(idx, inf_pop);
          break;
        }
      }
    }
  }
  //}
  
  /* mutateIndividualFromEliteGroup() method //{ */
  void Evolution::mutateIndividualFromEliteGroup(const IndividualConstPtr sup_ind, size_t idx, PopulationPtr inf_pop) {
    geometry_msgs::PosePtr new_pose = boost::make_shared<geometry_msgs::Pose>(*sup_ind->getPose());
    introduceNoise(new_pose, &mutation_distributions_);
    inf_pop->changeIndividualPose(new_pose, idx);
  }
  //}
  
  /* crossoverParts() method //{ */
  void Evolution::crossoverParts(IndividualConstPtr sup_ind1, IndividualConstPtr sup_ind2, size_t idx, PopulationPtr inf_pop) {
    geometry_msgs::PosePtr new_pose = boost::make_shared<geometry_msgs::Pose>(*sup_ind1->getPose());
    new_pose->orientation = sup_ind2->getPose()->orientation; 
    inf_pop->changeIndividualPose(new_pose, idx);
  }
  //}
  
  /* crossoverLinearCombination() method //{ */
  void Evolution::crossoverLinearCombination(IndividualConstPtr sup_ind1, IndividualConstPtr sup_ind2, size_t idx, PopulationPtr inf_pop){
    float lin_comb = lin_comb_dist_(gen_); // linear combination
    geometry_msgs::PosePtr new_pose = boost::make_shared<geometry_msgs::Pose>();
    geometry_msgs::PoseConstPtr ind1_pose = sup_ind1->getPose();
    geometry_msgs::PoseConstPtr ind2_pose = sup_ind2->getPose();
    new_pose->orientation = ind1_pose->orientation;
    
    // set yaw
    double ind1_yaw = getYaw(ind1_pose->orientation);
    double ind2_yaw = getYaw(ind2_pose->orientation);
    double new_yaw = lin_comb * ind1_yaw + (1-lin_comb) * ind2_yaw;
    setYaw(new_yaw, new_pose->orientation);

    // set position
    geometry_msgs::Point pos1 = ind1_pose->position;
    geometry_msgs::Point pos2 = ind2_pose->position;
    new_pose->position.x = lin_comb * pos1.x + (1-lin_comb) * pos2.x;
    new_pose->position.y = lin_comb * pos1.y + (1-lin_comb) * pos2.y;
    new_pose->position.z = pos1.z;
  
    inf_pop->changeIndividualPose(new_pose, idx);
  }
  //}
  
  /* reinitializeRP() method //{ */
  void Evolution::reinitializeRP(size_t idx, PopulationPtr inf_pop){
    geometry_msgs::PosePtr new_pose = boost::make_shared<geometry_msgs::Pose>(*initial_pose_);   
    introduceNoise(new_pose, &distributions_);
    inf_pop->changeIndividualPose(new_pose, idx);
  }
  //}
  
  //}

  /* computeRelativePoseOfUAV() method //{ */
  void Evolution::computeRelativePoseOfUAV(geometry_msgs::PoseConstPtr map_rel_pose) {
    geometry_msgs::PosePtr rel_pose_conv =
      convertPoseUnitsFromResToMetersWithZeroOriginNew(map_rel_pose, params_->uav_og_resolution);
    geometry_msgs::PosePtr lcp_conv =
    convertPoseUnitsFromResToMetersNew(loc_cur_p_, params_->uav_og_origin, params_->uav_og_resolution);
    geometry_msgs::PosePtr lrp_conv =
    convertPoseUnitsFromResToMetersNew(loc_rec_p_, params_->rec_og_origin, params_->rec_og_resolution);
    geometry_msgs::PosePtr inv_lcp = invertPose(lcp_conv);
    rec_uav_pose_ = compoundPose(inv_lcp, compoundPose(rel_pose_conv, lrp_conv));
  }
  
  //}

/* VISUALIZATION methods //{ */

  /* findConsistentCells() method //{ */
  /* support function //{ */
  
  bool isPointInsideMap(geometry_msgs::PointConstPtr p, unsigned width, unsigned height){
      if(p->x >= width || p->x < 0 || p->y >= height || p->y < 0){
        return false;
      }
      return true;
  }
  
  //}

  void Evolution::findConsistentCells(geometry_msgs::PoseConstPtr rel_pose) {
    consistent_cells_ = std::make_shared<OccupancyGridCells>();
 
    unsigned cur_map_width = cv_cur_->image.cols;
    unsigned cur_map_height = cv_cur_->image.rows;
    geometry_msgs::PointPtr transformed_cell;
    for(auto cell : *occupied_cells_rec_){
      transformed_cell = transformPointXY(rel_pose , cell);
      if(isPointInsideMap(transformed_cell, cur_map_width, cur_map_height)){
        float occupancyState = 
          static_cast<float>(
            cv_cur_->image.data[static_cast<int>(transformed_cell->y)*cur_map_width +
                                     static_cast<int>(transformed_cell->x)]);  
        if(occupancyState > occ_thr_img_){
          Cell2DPtr cell = std::make_shared<Cell2D>(static_cast<int>(transformed_cell->x), static_cast<int>(transformed_cell->y));
          consistent_cells_->push_back(cell);
        }
      }
    }
  }
  //}

  /* initRelPose() method //{ */
  geometry_msgs::PoseStampedPtr Evolution::initRelPose(geometry_msgs::PosePtr rel_pose) {
    geometry_msgs::PoseStampedPtr rel_pose_vis = boost::make_shared<geometry_msgs::PoseStamped>();
    // POSE INIT
    //rel_pose_vis->pose = *rel_pose;
    // Compounds relative pose of the other UAV with pose of current UAV -> This will return the pose of other UAV relative to the starting postition of the current UAV which is needed for correct placement of visualization of the other UAV pose in map.
    rel_pose_vis->pose =  *(compoundPose(loc_cur_p_, rel_pose));  
    // HEADER INIT
    rel_pose_vis->header.seq = 0;
    rel_pose_vis->header.stamp = params_->uav_og_stamp;
    rel_pose_vis->header.frame_id = params_->uav_og_frame_id; 

    return rel_pose_vis;
  }
  //}

  /* initPoseArray() method //{ */
  geometry_msgs::PoseArrayPtr Evolution::initPoseArray(GenerationPtr gen, float map_res) {
    geometry_msgs::PoseArrayPtr p_arr = boost::make_shared<geometry_msgs::PoseArray>();
    // Init poses
    std::vector<geometry_msgs::Pose> *poses = &(p_arr->poses);
    auto popul = gen->getPopulation();
    for(auto ind : *popul){
      geometry_msgs::Pose p = *(ind->getPose());
      p.position.x *= map_res;
      p.position.y *= map_res;
      p.position.z = 0;
     // double yaw = getYaw(p.orientation);
     // p.orientation.x = yaw;
      poses->push_back(p);
    }
     
    // Init header
    p_arr->header.seq = 0;
    p_arr->header.stamp = params_->rec_og_stamp;
    p_arr->header.frame_id = params_->rec_og_frame_id; 
    
  
    return p_arr;
  }
  //}

  /* initMarker() method //{ */
  visualization_msgs::MarkerPtr Evolution::initMarker(OccupancyGridCellsConstPtr og_cells, float map_res, int color) {
    geometry_msgs::PosePtr zero_pose = boost::make_shared<geometry_msgs::Pose>();
    zero_pose->position.x = 0;
    zero_pose->position.y = 0;
    zero_pose->position.z = 0;
    zero_pose->orientation.x = 0;
    zero_pose->orientation.y = 0;
    zero_pose->orientation.z = 0;
    zero_pose->orientation.w = 1;
    return initMarker(og_cells, zero_pose, map_res, color);
  }

    visualization_msgs::MarkerPtr Evolution::initMarker(OccupancyGridCellsConstPtr og_cells, geometry_msgs::PoseConstPtr best_pose, float map_res, int color) {
      visualization_msgs::MarkerPtr m = boost::make_shared<visualization_msgs::Marker>(); 
      // m->header.frame_id = "/uav1/local_origin";
      m->header.frame_id = params_->rec_og_frame_id;
      m->header.stamp = params_->rec_og_stamp;
      m->ns = "occupancy_cells";
      m->id = 0;
      m->type = visualization_msgs::Marker::POINTS;
      m->action = visualization_msgs::Marker::ADD;
      // compute markers transformation
      geometry_msgs::Pose* orig = &params_->uav_og_origin;
      m->pose.position.x = best_pose->position.x * map_res + orig->position.x;
      m->pose.position.y = best_pose->position.y * map_res + orig->position.y;
      m->pose.position.z = 0;
      double yaw = getYaw(best_pose->orientation);
      setRPY(0,0,yaw, m->pose.orientation);
      // m->pose.orientation.x = best_pose->orientation.x;
      // m->pose.orientation.y = best_pose->orientation.y;
      // m->pose.orientation.z = best_pose->orientation.z;
      // m->pose.orientation.w = best_pose->orientation.w;
      m->scale.x = 5*map_res; 
      m->scale.y = 5*map_res;
      m->color.a = 1.0; // alpha
      m->color.r = 0.0;
      m->color.g = 0.0;
      m->color.b = 0.0;
      if(color == 0)
        m->color.g = 1.0;
      else if(color == 1)
        m->color.r = 1.0;
      else
        m->color.b = 1.0;

      for(auto cell : *og_cells) {
        geometry_msgs::Point p;
        p.x = cell->col * map_res; 
        p.y = cell->row * map_res;
        p.z = 0;
        m->points.push_back(p);
      }

      return m;
    }
  //}

    /* sendVisualization() method //{ */
    void Evolution::sendVisualization(geometry_msgs::PoseStampedPtr rel_pose, geometry_msgs::PoseArrayConstPtr poses, visualization_msgs::MarkerConstPtr occup_cells_rec, visualization_msgs::MarkerConstPtr occup_cells_cur, visualization_msgs::MarkerConstPtr consistent_cells) {
      //ROS_INFO_STREAM("Pose position of received marker msg: " << occup_cells_rec->pose.position);
      //ROS_INFO_STREAM("First pose position in PoseArray msg: " << poses->poses[0].position);
    
      try {
        params_->pub_gen_poses.publish(*poses);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", params_->pub_gen_poses.getTopic().c_str());
      }
    
      try {
        params_->pub_occup_cells_rec.publish(*occup_cells_rec);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", params_->pub_occup_cells_rec.getTopic().c_str());
      }
    
      try {
        params_->pub_occup_cells_cur.publish(*occup_cells_cur);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", params_->pub_occup_cells_rec.getTopic().c_str());
      }
      
      try {
        params_->pub_consistent_cells.publish(*consistent_cells);
      }
      catch (...) {
        ROS_ERROR("Exception caught during publishing topic %s.", params_->pub_occup_cells_rec.getTopic().c_str());
      }
    }
    //}

  /* updateRelPose() method //{ */
  void Evolution::updateRelPose(geometry_msgs::PoseStampedPtr rel_pose_vis, geometry_msgs::PoseConstPtr rel_pose) {
    // POSE
    geometry_msgs::PosePtr lcp_conv =
    convertPoseUnitsFromResToMetersNew(loc_cur_p_, params_->uav_og_origin, params_->uav_og_resolution);
    geometry_msgs::PosePtr originDifference = compoundPose(invertPose(params_->rec_og_origin), params_->uav_og_origin);
    rel_pose_vis->pose = *(compoundPose(originDifference, compoundPose(lcp_conv, rel_pose)));

    // HEADER INIT
    ++rel_pose_vis->header.seq;
    rel_pose_vis->header.stamp = params_->uav_og_stamp; // the stamp is still same, but like this it can be used with updated occupancy grid map
  }
  //}

    /* updatePoseArray() method //{ */
    void Evolution::updatePoseArray(geometry_msgs::PoseArrayPtr p_arr, GenerationPtr gen, float map_res){
      // PoseArray update
      std::vector<geometry_msgs::Pose> *poses = &(p_arr->poses);
      auto popul = gen->getPopulation();
      size_t p_size = popul->size();
      for(size_t i = 0; i < p_size; ++i){
        geometry_msgs::Pose p = *(popul->at(i)->getPose());
        p.position.x *= map_res;
        p.position.y *= map_res;
        p.position.z = 0; // does not need this one
        // double yaw = getYaw(p.orientation);
        // p.orientation.x = yaw;
        poses->at(i) = p; 
      }
      p_arr->header.stamp = params_->rec_og_stamp;
      // p_arr->header.stamp = ros::Time::now();
    }
    //}

    /* updateOccupiedCells() method //{ */
    void Evolution::updateOccupiedCells(visualization_msgs::MarkerPtr occup_cells, float map_res, geometry_msgs::PoseConstPtr new_pose) {
      // Marker update
      geometry_msgs::Pose* orig = &params_->uav_og_origin;
      occup_cells->pose.position.x = new_pose->position.x * map_res + orig->position.x;
      occup_cells->pose.position.y = new_pose->position.y * map_res + orig->position.y;
      occup_cells->pose.orientation = new_pose->orientation;
      // occup_cells->header.stamp = ros::Time::now();
      occup_cells->header.stamp = params_->rec_og_stamp;
    }
    //}
    
    /* updateVisualization() method //{ */
    void Evolution::updateVisualization(geometry_msgs::PoseStampedPtr rel_pose_vis, geometry_msgs::PoseArrayPtr p_arr, visualization_msgs::MarkerPtr occup_cells_rec, visualization_msgs::MarkerPtr occup_cells_cur, visualization_msgs::MarkerPtr consist_cells, GenerationPtr gen, float map_res, float cur_map_res) {
      // This will work only if the size of each generation is the same
      geometry_msgs::PoseConstPtr best_pose = gen->getBestPose();
    
      updateRelPose(rel_pose_vis, rec_uav_pose_);
      updatePoseArray(p_arr, gen, map_res);
      updateOccupiedCells(occup_cells_rec, map_res, best_pose);
      updateOccupiedCells(occup_cells_cur, cur_map_res, boost::make_shared<geometry_msgs::Pose>());
      updateOccupiedCells(consist_cells, cur_map_res, boost::make_shared<geometry_msgs::Pose>());
    }
    //}

//}

  /* SUPPORT FUNCTIONS //{ */
 
  void Evolution::setErrorRanges(float orientation_err, float position_err){
    orientation_error_range_ = orientation_err;
    position_error_range_ = position_err;
  }
  
  void Evolution::convertPoseUnitsFromResToMeters(geometry_msgs::PosePtr pose, const geometry_msgs::Pose& origin, float res){
    geometry_msgs::Point* p = &(pose->position);
    p->x = p->x * res + origin.position.x;
    p->y = p->y * res + origin.position.y;
  }

  geometry_msgs::PosePtr Evolution::convertPoseUnitsFromResToMetersNew(const geometry_msgs::PoseConstPtr pose, const geometry_msgs::Pose& origin, float res) const {
    geometry_msgs::PosePtr ret = boost::make_shared<geometry_msgs::Pose>(*pose);
    geometry_msgs::Point* p = &(ret->position);
    p->x = p->x * res + origin.position.x;
    p->y = p->y * res + origin.position.y;

    return ret;
  }

  geometry_msgs::PosePtr Evolution::convertPoseUnitsFromResToMetersWithZeroOriginNew(const geometry_msgs::PoseConstPtr pose, float res) const {
    geometry_msgs::PosePtr ret = boost::make_shared<geometry_msgs::Pose>(*pose);
    geometry_msgs::Point* p = &(ret->position);
    p->x = p->x * res;
    p->y = p->y * res;

    return ret;
  }

  /* quaternionToEulerAngles() method //{ */

//  /// Not needed because tf has function getRPY (roll, pitch, yaw)
//  EulerAngles Evolution::quaternionToEulerAngles(geometry_msgs::Quaternion q){
//    EulerAngles angles;
//
//    // roll (x-axis rotation)
//    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
//    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
//    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
//
//    // pitch (y-axis rotation)
//    double sinp = 2 * (q.w * q.y - q.z * q.x);
//    if (std::abs(sinp) >= 1)
//        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//    else
//        angles.pitch = std::asin(sinp);
//
//    // yaw (z-axis rotation)
//    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
//    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
//    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
//
//    return angles;
//  }

  //}
  
  //}

  /* DEBUG FUNCTIONS //{ */
  /* printValuesInCvImage() //{ */
  void Evolution::printValuesInCvImage(cv_bridge::CvImageConstPtr img_map){
    ROS_INFO("[OccupancyGridMapMerge]: Starting printing values");     
    unsigned width = img_map->image.cols;
    unsigned height = img_map->image.rows;
  
    ROS_INFO("[OccupancyGridimg_mapMerge]: for cycle");     
    for(unsigned h = 0; h < height;++h){
      for(unsigned w = 0; w < width; ++w){
        /* print all values */
        printf("%3d ", img_map->image.data[w + h*width]);
      }
      printf("\n");
    }
  }
  //}
  
  //}
  
} //namespace evolution

} // namespace merger2d
