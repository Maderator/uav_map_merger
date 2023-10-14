/**  @file
 *   @author Jan Maděra <maderja@fel.cvut.cz> (janmadera97@gmail.com)
 *   @date 2020
 *   @copyright The 3-Clause BDS License
 */

#include "Merger2d.h"

namespace merger2d{

  /* CONSTRUCTORS //{ */  
  Merger2d::Merger2d(){}

  /* ONLY LOCAL CUR POSE KNOWN //{ */
  
  Merger2d::Merger2d(
      nav_msgs::OccupancyGridConstPtr cur_map,
      nav_msgs::OccupancyGridConstPtr rec_map,
      geometry_msgs::PoseConstPtr loc_cur_p,
      EvolutionParams params
      ) : params_(params) 
  {
    // copy local current pose constPtr to member variable
    loc_cur_p_ = boost::make_shared<geometry_msgs::Pose>(*loc_cur_p);
    
    occupied_cells_cur_ = std::make_shared<std::vector<Cell2DPtr>>();
    occupied_cells_rec_ = std::make_shared<std::vector<Cell2DPtr>>();
    cv_cur_ = boost::make_shared<cv_bridge::CvImage>();
    cv_rec_ = boost::make_shared<cv_bridge::CvImage>();
  
    // merger has uninitialized poses
    has_glob_c_ = false;
    has_loc_r_  = false;
    has_glob_r_ = false;


    
    setMemberVars(cur_map, rec_map);
  }

  //}

  /* BOTH LOCAL POSES KNOWN //{ */
  
  Merger2d::Merger2d(
      nav_msgs::OccupancyGridConstPtr cur_map,
      nav_msgs::OccupancyGridConstPtr rec_map,
      geometry_msgs::PoseConstPtr loc_cur_p,
      geometry_msgs::PoseConstPtr loc_rec_p,
      EvolutionParams params
      ) : params_(params)
  {
    // copy local current pose constPtr to member variable
    loc_cur_p_ = boost::make_shared<geometry_msgs::Pose>(*loc_cur_p);
    loc_rec_p_ = boost::make_shared<geometry_msgs::Pose>(*loc_rec_p);

    occupied_cells_cur_ = std::make_shared<std::vector<Cell2DPtr>>();
    occupied_cells_rec_ = std::make_shared<std::vector<Cell2DPtr>>();
    cv_cur_ = boost::make_shared<cv_bridge::CvImage>();
    cv_rec_ = boost::make_shared<cv_bridge::CvImage>();
    
    // merger has uninitialized global poses
    has_glob_c_ = false;
    has_glob_r_ = false;
    has_loc_r_  = true;
    
    setMemberVars(cur_map, rec_map);
  }
  
  //}
  
  /* ALL POSES KNOWN //{ */
  
  Merger2d::Merger2d(
      nav_msgs::OccupancyGridConstPtr cur_map, 
      nav_msgs::OccupancyGridConstPtr rec_map,
      geometry_msgs::PoseConstPtr loc_cur_p,
      geometry_msgs::PoseConstPtr glob_cur_p,
      geometry_msgs::PoseConstPtr loc_rec_p,
      geometry_msgs::PoseConstPtr glob_rec_p,
      EvolutionParams params
      ) : params_(params)
  {
    // copy local current pose constPtr to member variable
    loc_cur_p_ = boost::make_shared<geometry_msgs::Pose>(*loc_cur_p);
    loc_rec_p_ = boost::make_shared<geometry_msgs::Pose>(*loc_rec_p);
    glob_cur_p_ = boost::make_shared<geometry_msgs::Pose>(*glob_cur_p);
    glob_rec_p_ = boost::make_shared<geometry_msgs::Pose>(*glob_rec_p);
  
    occupied_cells_cur_ = std::make_shared<std::vector<Cell2DPtr>>();
    occupied_cells_rec_ = std::make_shared<std::vector<Cell2DPtr>>();
    cv_cur_ = boost::make_shared<cv_bridge::CvImage>();
    cv_rec_ = boost::make_shared<cv_bridge::CvImage>();
    
    // merger has all poses initialized
    has_glob_c_ = true;
    has_glob_r_ = true;
    has_loc_r_  = true;
  
    setMemberVars(cur_map, rec_map);
  }
  
  //}

  /* setMemberVars() method //{ */
  void Merger2d::setMemberVars(nav_msgs::OccupancyGridConstPtr cur_map, nav_msgs::OccupancyGridConstPtr rec_map){
 
    // Estimate of max error in yaw angle and position of pose
    uint32_t m_size = std::max(std::max(cur_map->info.height, cur_map->info.width), std::max(rec_map->info.height, rec_map->info.width));
    setErrorRanges(M_PI * (*(params_.err_angle)), m_size/2 * (*(params_.err_range)));   
  
    initGUI(*(params_.gui));
    // store OG pointers
    og_cur_ = cur_map;
    og_rec_ = rec_map;



    // convert OccupancyGrids to CV images
    *cv_cur_ = *convertOGToCvImg(cur_map);
    *cv_rec_ = *convertRecOGToCvImg(rec_map, og_cur_->info.width, og_cur_->info.height);
    *occupied_cells_cur_ = *findOccupiedCells(og_cur_);
    *occupied_cells_rec_ = *findOccupiedCells(og_rec_);
  }
  //}

  //}

/* GETTERS AND SETTERS //{ */

  cv_bridge::CvImagePtr Merger2d::getCurrentCvMap() const { return cv_cur_; }

  void Merger2d::setCurrentMap(nav_msgs::OccupancyGridConstPtr new_c_map) {
    og_cur_ = new_c_map;
    cv_cur_ = convertOGToCvImg(new_c_map);
  }
  
  cv_bridge::CvImagePtr Merger2d::getReceivedCvMap() const { return cv_rec_; } 

  void Merger2d::setReceivedMap(nav_msgs::OccupancyGridConstPtr new_r_map) {
    og_rec_ = new_r_map;
    cv_rec_ = convertOGToCvImg(new_r_map); 
  }
  
  float Merger2d::getAcceptanceIndex() const { return accept_index_; }
  
  geometry_msgs::PoseConstPtr Merger2d::getRelativeMapPose() const { 
    return rel_map_pose_; 
  }
  
  geometry_msgs::PoseConstPtr Merger2d::getLocalCurrentPosition() const { return loc_cur_p_; }
 
  void Merger2d::setLocalCurrentPosition(geometry_msgs::PoseConstPtr new_lcp) 
  { loc_cur_p_ = boost::make_shared<geometry_msgs::Pose>(*new_lcp); }
  
  geometry_msgs::PoseConstPtr Merger2d::getGlobalCurrentPosition() const { 
    if(!glob_cur_p_)
      ROS_WARN("[OccupancyGridMerger]: Global position of current drone is not defined!Returning null pointer.");
    return glob_cur_p_; 
  }
  
  void Merger2d::setGlobalCurrentPosition(geometry_msgs::PoseConstPtr new_gcp) 
  { *glob_cur_p_ = *new_gcp; }
 
  geometry_msgs::PoseConstPtr Merger2d::getLocalRecievedPosition() const { 
    if(!glob_rec_p_)
      ROS_WARN("[OccupancyGridMerger]: Local position of other drone is not defined! Returning null pointer.");
    return loc_rec_p_; }
  
  void Merger2d::setLocalRecievedPosition(geometry_msgs::PoseConstPtr new_lrp) 
  { *loc_rec_p_ = *new_lrp; }

  geometry_msgs::PoseConstPtr Merger2d::getGlobalRecievedPosition() const { 
    if(!glob_rec_p_)
      ROS_WARN("[OccupancyGridMerger]: Global position of other drone is not defined!Returning null pointer.");
    return glob_rec_p_; 
  }

  void Merger2d::setGlobalRecievedPosition(geometry_msgs::PoseConstPtr new_grp)
  { *glob_rec_p_ = *new_grp; }

  EvolutionParams Merger2d::getEvolutionparams() const { return params_; }
  
  void Merger2d::setEvolutionParams(EvolutionParams params) { params_ = params; }

//}

  /* initializeEvolver() method //{ */
  void Merger2d::initializeEvolver() {
    ROS_ASSERT_MSG(loc_cur_p_, "loc_cur_p_ missing in Merger2d");
  
    initializeUnreceivedPoses();
    float occ_thr_img = IMG_ZERO + *(params_.occup_thr) * (IMG_MAX_OCCUP_VAL - IMG_ZERO); // convert threshold from percentage value to image cell value
    convertPosesUnitsFromMetersToResolution();
    evolver_ = evolution::Evolution(&params_, cv_cur_, cv_rec_, 
        occupied_cells_cur_, occupied_cells_rec_,
        loc_cur_p_, glob_cur_p_, loc_rec_p_, glob_rec_p_, 
        occ_thr_img, orientation_error_range_, position_error_range_);
  }
  //}

  /* initializeGAGeneration() method //{ */
  void Merger2d::initializeGAGeneration() {
    evolution_generation_ = evolver_.initializeGAGeneration( *(params_.pop_size), *(params_.accept_thr));
  }
  //}
  
/* mergeMapsContinuously() method //{ */
  nav_msgs::OccupancyGridPtr Merger2d::mergeMapsContinuously() {
    // evolve
    unsigned min_ms = *(params_.min_mic_sec);
    unsigned max_ms = *(params_.max_mic_sec);

    *occupied_cells_cur_ = *findOccupiedCells(og_cur_);
    *occupied_cells_rec_ = *findOccupiedCells(og_rec_);
    evolver_.evolve(min_ms, max_ms, evolution_generation_);
    rel_map_pose_ = convertPoseUnitsFromResToMetersWithZeroOriginNew(evolution_generation_->getBestPose(), og_rec_->info.resolution);
    transformCvImageByPose(cv_rec_, evolution_generation_->getBestPose(), og_rec_->info.origin,*(params_.gui));
    cv_bridge::CvImagePtr new_cv_map = blendMaps(cv_cur_, cv_rec_);
    nav_msgs::OccupancyGridPtr merged_map = 
      convertCvImgToOG(new_cv_map, og_cur_);
    OccupancyGridCellsPtr merged_cells = mergeCells(evolution_generation_->getBestPose());
 
    return merged_map;
  }
//}

/* findOptimalRelativePose() methods //{ */
  geometry_msgs::PoseConstPtr Merger2d::findOptimalRelativePose(TimerMergingInfo& prev_merg_info){
    ROS_ASSERT_MSG(loc_cur_p_, "loc_cur_p_ missing in Merger2d");

    initializeUnreceivedPoses();
    float occ_thr_img = IMG_ZERO + *(params_.occup_thr) * (IMG_MAX_OCCUP_VAL - IMG_ZERO); // convert threshold from percentage value to image cell value
    geometry_msgs::PosePtr prev_rel_pose = boost::make_shared<geometry_msgs::Pose>(*prev_merg_info.previous_rel_pose);
    geometry_msgs::PosePtr prev_loc_cur_p = boost::make_shared<geometry_msgs::Pose>(*prev_merg_info.prev_loc_cur_p);
    geometry_msgs::PosePtr prev_loc_rec_p = boost::make_shared<geometry_msgs::Pose>(*prev_merg_info.prev_loc_rec_p);
    convertPosesAndPrevPosesUnitsFromMetersToResolution(prev_rel_pose, prev_loc_cur_p, prev_loc_rec_p);

    // Decide whether the previous map merging was successful or not and set the orientation and position error accordingly
    computeErrorRanges(prev_merg_info.previous_acceptance_index);

    evolution::Evolution evolver(&params_, cv_cur_, cv_rec_, 
        occupied_cells_cur_, occupied_cells_rec_,
        loc_cur_p_, loc_rec_p_, 
        occ_thr_img, orientation_error_range_, position_error_range_, prev_rel_pose, prev_loc_cur_p, prev_loc_rec_p);
  
    // Init generation
    evolution::GenerationPtr gen = evolver.initializeGAGeneration( *(params_.pop_size), *(params_.accept_thr));

    // evolve
    unsigned min_ms = *(params_.min_mic_sec);// 1000;
    unsigned max_ms = *(params_.max_mic_sec); // 30000;
    evolver.evolve(min_ms, max_ms, gen);

    geometry_msgs::PoseConstPtr rel_pose = gen->getBestPose();

    return rel_pose;
  }

  /* initializeUnreceivedPoses() method  //{ */
  
  void Merger2d::initializeUnreceivedPoses(){
    if(!has_loc_r_){
      ROS_WARN("The local pose of second UAV is not known!!! This can cause faulty merging");
      loc_rec_p_ = boost::make_shared<geometry_msgs::Pose>(*loc_cur_p_);
    }
    if(!has_glob_c_ || !has_glob_r_){
      glob_cur_p_ = boost::make_shared<geometry_msgs::Pose>(*loc_cur_p_);
      glob_rec_p_ = boost::make_shared<geometry_msgs::Pose>(*loc_rec_p_);
    }
  }
  
  //}
//}

  /* findOccupiedCells() method //{ */
  OccupancyGridCellsPtr Merger2d::findOccupiedCells(nav_msgs::OccupancyGridConstPtr og){
    int w = og->info.width;
    int h = og->info.height;
    const std::vector<int8_t>* data = &(og->data);
    OccupancyGridCellsPtr cells = std::make_shared<std::vector<Cell2DPtr>>();

    int occup_thr =  *(params_.occup_thr) * 100; // og has values in range [0,100] (not counting unknown cells with -1) so multiply by 100

    // OLD PLAIN FOR
    for(int row = 0; row < h; ++row) {
      int el_before = row*w; // number of elements in rows before current row
      for(int col = 0; col < w; ++col) {
        if((*data)[col + el_before] > occup_thr)
          cells->push_back(std::make_shared<Cell2D>(col,row));
      }
    }
    // move semantics is used above c++11
    return cells;
  }
  //}

  bool isPointInsideMap(geometry_msgs::PointConstPtr p, unsigned width, unsigned height){
      if(p->x >= width || p->x < 0 || p->y >= height || p->y < 0){
        return false;
      }
      return true;
  }

  /* mergeCells() method //{ */
  OccupancyGridCellsPtr Merger2d::mergeCells(geometry_msgs::PoseConstPtr pose) {
    OccupancyGridCellsPtr merged_cells = std::make_shared<OccupancyGridCells>(*occupied_cells_cur_);
    unsigned cur_map_width = cv_cur_->image.cols;
    unsigned cur_map_height = cv_cur_->image.rows;
    geometry_msgs::PointPtr transformed_cell;
    for(auto cell : *occupied_cells_rec_){
      transformed_cell = transformPointXY(pose , cell);
      if(isPointInsideMap(transformed_cell, cur_map_width, cur_map_height)){
        int x = static_cast<int>(transformed_cell->x);
        int y = static_cast<int>(transformed_cell->y);
        Cell2DPtr new_cell = std::make_shared<Cell2D>(x,y);
        merged_cells->push_back(new_cell);
      }
    }
    return merged_cells;
  }
  //}

  /* setLocalPoses() method //{ */
  void Merger2d::setLocalPoses(geometry_msgs::PoseConstPtr cur_loc_p, geometry_msgs::PoseConstPtr rec_loc_p) {
    loc_cur_p_ = boost::make_shared<geometry_msgs::Pose>(*cur_loc_p);
    loc_rec_p_ = boost::make_shared<geometry_msgs::Pose>(*rec_loc_p);
  }
  //}

  /* computeAcceptanceIndex() method //{ */
  void Merger2d::computeAcceptanceIndex(long unsigned agree, long unsigned disagree){
    //ROS_INFO_STREAM("ComputeAcceptanceIndex: agree=" << agree << " disagree=" << disagree);
    if(agree == 0){
      accept_index_ = 0;
    } else{
      accept_index_ = static_cast<float>(agree) / (agree + disagree);
    }
  }
  //}

  /* convertPosesUnitsFromMetersToResolution() method //{ */
  
  void Merger2d::convertPosesUnitsFromMetersToResolution() {
    convertPoseUnitsToRes(loc_cur_p_, og_cur_);
    convertPoseUnitsToRes(glob_cur_p_, og_cur_);
    convertPoseUnitsToRes(loc_rec_p_, og_rec_);
    convertPoseUnitsToRes(glob_rec_p_, og_rec_);
  }

  void Merger2d::convertPosesAndPrevPosesUnitsFromMetersToResolution(geometry_msgs::PosePtr prev_rel_pose, geometry_msgs::PosePtr prev_loc_cur_p, geometry_msgs::PosePtr prev_loc_rec_p) {
    convertPoseUnitsToRes(prev_rel_pose, og_rec_);
    convertPoseUnitsToRes(prev_loc_cur_p, og_cur_);
    convertPoseUnitsToRes(prev_loc_rec_p, og_rec_);
    convertPosesUnitsFromMetersToResolution();
  }
  //}

  /* convertPoseUnitsToRes() method //{ */
  // what if the two maps have different resolution? -> then it is ok in this case. But change was made in final blending of maps
  void Merger2d::convertPoseUnitsToRes(geometry_msgs::PosePtr pose, nav_msgs::OccupancyGridConstPtr og) {
    geometry_msgs::Point* p = &(pose->position);
    const geometry_msgs::Point* origin = &(og->info.origin.position);
    float res = og->info.resolution;
    p->x = (p->x - origin->x) / res;
    p->y = (p->y - origin->y) / res;
  }
  //}

  /* convertPoseUnitsFromResToMeters() method //{ */
  void Merger2d::convertPoseUnitsFromResToMeters(geometry_msgs::PosePtr pose, nav_msgs::OccupancyGridConstPtr og){
    geometry_msgs::Point* p = &(pose->position);
    const geometry_msgs::Point* origin = &(og->info.origin.position);
    float res = og->info.resolution;
    p->x = p->x * res + origin->x;
    p->y = p->y * res + origin->y;
  }

  geometry_msgs::PosePtr Merger2d::convertPoseUnitsFromResToMeters(const geometry_msgs::PoseConstPtr pose, const nav_msgs::OccupancyGridConstPtr og) const {
    geometry_msgs::PosePtr ret = boost::make_shared<geometry_msgs::Pose>(*pose);
    geometry_msgs::Point* p = &(ret->position);
    const geometry_msgs::Point* origin = &(og->info.origin.position);
    float res = og->info.resolution;
    p->x = p->x * res + origin->x;
    p->y = p->y * res + origin->y;

    return ret;
  }
  //}
  
  /* convertPoseUnitsFromResToMetersWithZeroOriginNew() method //{ */
  geometry_msgs::PosePtr Merger2d::convertPoseUnitsFromResToMetersWithZeroOriginNew(const geometry_msgs::PoseConstPtr pose, float res) const {
    geometry_msgs::PosePtr ret = boost::make_shared<geometry_msgs::Pose>(*pose);
    geometry_msgs::Point* p = &(ret->position);
    p->x = p->x * res;
    p->y = p->y * res;
  
    return ret;
  }
  //}

/* blendMaps() method //{ */

  /* SUPPORT METHODS //{ */
  void Merger2d::blendSameSizeImgs(cv::Mat img1, cv::Mat img2, bool unknown_thr){
    // variables for computation of acceptance index
    long unsigned agree = 0, disagree = 0;
  
    int idx, max_r_idx;
    for (int ri = 0; ri < img1.rows; ri++)
    {
      idx = ri * img1.cols;
      max_r_idx = idx + img1.cols;
      for (int ci = idx; ci < max_r_idx; ci++)
      {
        bool is_unknown = img1.data[ci] <= unknown_thr || img2.data[ci] <= unknown_thr;
        if(!is_unknown && img1.data[ci] == img2.data[ci]){
          ++agree; 
        }
        else{
          if(img1.data[ci] < img2.data[ci]){
            img1.data[ci] = img2.data[ci];
          }
          if(!is_unknown){
            ++disagree;
          }
        }
      }
    }
    computeAcceptanceIndex(agree, disagree);
  }
  
  void Merger2d::blendDifferentSizeImgs(cv::Mat img1, cv::Mat img2, bool unknown_thr){
    // variables for computation of acceptance index
    long unsigned agree = 0, disagree = 0;
  
    int rows = img1.rows < img2.rows ? img1.rows : img2.rows;
    int cols = img1.cols < img2.cols ? img1.cols : img2.cols;

    for (int ri = 0; ri < rows; ri++)
    {
      int idx1 = ri * img1.cols;
      int idx2 = ri * img2.cols; 

      for (int ci = 0; ci < cols; ci++)
      {
        bool is_unknown = img1.data[idx1] <= unknown_thr || img2.data[idx2] <= unknown_thr;
        if(!is_unknown && img1.data[idx1] == img2.data[idx2]){
          ++agree;
        }
        else{ 
          if(img1.data[idx1] < img2.data[idx2]){
            img1.data[idx1] = img2.data[idx2];
          }
          if(!is_unknown){
            ++disagree;
          }
        }

        ++idx1;
        ++idx2;
      }
    }
    computeAcceptanceIndex(agree, disagree);
  }
  //}

  cv_bridge::CvImagePtr Merger2d::blendMaps(cv_bridge::CvImagePtr map1, const cv_bridge::CvImagePtr map2){
    cv::Mat img1 = map1->image;
    cv::Mat img2 = map2->image;

    /// Threshold of cells being unknown in image
    int unknown_thr = IMG_UNKNOWN + (((IMG_ZERO) - (IMG_UNKNOWN))/2);

    if(img1.rows == img2.rows || img1.cols == img2.cols){
      blendSameSizeImgs(img1, img2, unknown_thr);
    } else {
      ROS_WARN("[%s]: Maps dimensions have different sizes!", ros::this_node::getName().c_str());
      blendDifferentSizeImgs(img1, img2, unknown_thr);
    }

    return map1;
  }
  
//}

/* CONVERSION OG<->IMG //{ */

  /* convertOGToCvImg() method //{ */
  cv_bridge::CvImagePtr Merger2d::convertOGToCvImg(const nav_msgs::OccupancyGridConstPtr map){
    sensor_msgs::ImagePtr img_p = convertOGToImg(map);
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(img_p); 
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("[OccupancyGridMapMerge]: cv_bridge exception: %s", e.what());
      return nullptr;
    }

    return cv_ptr;
  }

  cv_bridge::CvImagePtr Merger2d::convertOGToCvImg(const nav_msgs::OccupancyGridConstPtr map, unsigned cols, unsigned rows){
    sensor_msgs::ImagePtr img_p = convertOGToImg(map, cols, rows);
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(img_p); 
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("[OccupancyGridMapMerge]: cv_bridge exception: %s", e.what());
      return nullptr;
    }

    return cv_ptr;
  }

  cv_bridge::CvImagePtr Merger2d::convertRecOGToCvImg(const nav_msgs::OccupancyGridConstPtr rec_map, unsigned cur_cols, unsigned cur_rows){
    unsigned width = cur_cols > rec_map->info.width ? cur_cols : rec_map->info.width;
    unsigned height = cur_rows > og_rec_->info.height ? cur_rows : og_rec_->info.height;

    return convertOGToCvImg(rec_map, width, height);
  }
  //}

  /* convertOGToImg() method //{ */
  sensor_msgs::ImagePtr Merger2d::convertOGToImg(const nav_msgs::OccupancyGridConstPtr map){
    return convertOGToImg(map, map->info.width, map->info.height);
  }

  sensor_msgs::ImagePtr Merger2d::convertOGToImg(const nav_msgs::OccupancyGridConstPtr map, unsigned cols, unsigned rows){
    uint32_t height = map->info.height;       
    uint32_t width = map->info.width;

    if(cols < width || rows < height)
      ROS_ERROR_ONCE("[OccupancyGridMerger]: given width or height is smaller than dimensions of given map");

    sensor_msgs::ImagePtr img_p = boost::make_shared<sensor_msgs::Image>();
    /* header */
    img_p->header = map->header;

    /* info */
    img_p->height = rows;
    img_p->width = cols;

    img_p->encoding = sensor_msgs::image_encodings::MONO8; /* 0-256 */
    img_p->is_bigendian = false;
    img_p->step = cols;

    /* data conversion to pixel color values */
    for(uint32_t ri = 0; ri < height; ++ri){
      uint32_t i_before = ri*width;
      // fill row with data from occupancy grid
      for(uint32_t ci = 0; ci < width; ++ci){
        if(map->data[i_before + ci] == -1){
          img_p->data.push_back(IMG_UNKNOWN);
        }else if(map->data[i_before + ci] == 0){
          img_p->data.push_back(IMG_ZERO);
        }else if(map->data[i_before + ci] == 100){
          img_p->data.push_back(IMG_MAX_OCCUP_VAL);
        }
      }
      // fill the rest of the row
      for(uint32_t ci = width; ci < cols; ++ci){
        img_p->data.push_back(IMG_UNKNOWN);
      }
    }
    // fill the rest of the matrix rows
    uint32_t count = cols*rows;
    for(uint32_t i = cols*height; i < count; ++i){
        img_p->data.push_back(IMG_UNKNOWN);
    }

    return img_p;
  }
  //}

  /* convertCvImgToImg() method //{ */
  
  sensor_msgs::ImagePtr Merger2d::convertCvImgToImg(
      const cv_bridge::CvImageConstPtr cv_img_p){
    return cv_img_p->toImageMsg();
  }
  
  //}

  /* convertCvImgToOG() method //{ */
  nav_msgs::OccupancyGridPtr Merger2d::convertCvImgToOG(
      const cv_bridge::CvImageConstPtr img_p, 
      const nav_msgs::OccupancyGridConstPtr cur_og){ 
    float occ_thr = *(params_.occup_thr);

    if(occ_thr < 0.0){
      ROS_WARN("[OccupancyGridMerger]: Occupancy threshold is %f  but it has to be bigger or equal to 0!!! Setting occ_thr to 0.0", occ_thr);
      occ_thr = 0.0;
    } else if(occ_thr > 1.0){
      ROS_WARN("[OccupancyGridMerger]: Occupancy threshold is %f  but it has to be smaller or equal to 1!!! Setting occ_thr to 1.0", occ_thr);
      occ_thr = 1.0;
    }
    
    nav_msgs::OccupancyGridPtr map_p = boost::make_shared<nav_msgs::OccupancyGrid>();
    
    /* header */
    map_p->header = cur_og->header;

    /* info */
    map_p->info = cur_og->info;
    
    uint32_t height = img_p->image.rows;
    uint32_t width = img_p->image.cols;

    // Check if width and height are correct
    if(height != map_p->info.height || width != map_p->info.width){
      ROS_ERROR("[OccupancyGridMerge]: Image height or width differs from given height or width while converting image to occupancy grid");
    }

    // compute threshold from percentage value of occ_thr in interval 
    // (0.0-1.0) to [0-255] color value
    int threshold = IMG_ZERO + (IMG_MAX_OCCUP_VAL-IMG_ZERO)*occ_thr;

    /* data */
    uint32_t count = width*height;
    for(uint32_t i = 0; i < count; ++i){
      if(img_p->image.data[i] == 0){
        map_p->data.push_back(-1);
      }else if(img_p->image.data[i] < threshold){
        map_p->data.push_back(0);
      }else if(img_p->image.data[i] >= threshold){
        map_p->data.push_back(100);
      }else{
        ROS_INFO("[OccupancyGridMapMerge]: Image contains data: %d", img_p->image.data[i]);     
        map_p->data.push_back(-1);
      }
    }


    return map_p;
  }
  //}
  
//}
 
  /* ERROR METHODS //{ */
  
  void Merger2d::computeErrorRanges(float prev_accept_index) {
    // THIS MULTIPLIER DETERMINE HOW BIG WILL THE ERROR BE
   // float multiplier;
   // float up_b = 0.9; // upper boundary of searching the whole occupancy grid for best pose

   // if(prev_accept_index < up_b){
   //   multiplier = 1;
   // }else if(prev_accept_index > 0.99){
   //   multiplier = 0.01;
   // } else {
   //   multiplier = cos(11 * (prev_accept_index-up_b) * M_PI_2); // This only work for up_b = 0.9
   // }

    uint32_t m_size = std::max(std::max(og_cur_->info.height, og_cur_->info.width), std::max(og_rec_->info.height, og_rec_->info.width));
    float max_pos_err = m_size/2 * (*(params_.err_range));
    float max_ori_err = M_PI * (*(params_.err_angle));
    setErrorRanges(max_ori_err, max_pos_err);   
  }
  
  void Merger2d::setErrorRanges(float orientation_err, float position_err){
    orientation_error_range_ = orientation_err;
    position_error_range_ = position_err;
  }
  
  //}

  /* SUPPORT FUNCTIONS //{ */

  void Merger2d::initGUI(bool gui){
    if (gui){ 
      int flags = cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED;
      //cv::namedWindow("curmap", flags);
      //cv::namedWindow("recmap", flags);
      //cv::namedWindow("merged_map", flags);
    }
  }

  /* quaternionToEulerAngles() method //{ */

//  /// Not needed because tf has function getRPY (roll, pitch, yaw)
//  EulerAngles Merger2d::quaternionToEulerAngles(geometry_msgs::Quaternion q){
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

  /* printOccupiedCells() method //{ */
  void Merger2d::printOccupiedCells(OccupancyGridCellsPtr cells){
    printf("Printing cells of OG: \n");
    for(auto it : *cells){
      printf("[%d, %d]\n", it->col, it->row);
    }
    printf("%lu occupied cells\n", cells->size());
  }
  //}

  /* printValuesInOccupancyGridMap() method //{ */
  void Merger2d::printValuesInOccupancyGridMap(nav_msgs::OccupancyGridConstPtr map){
    ROS_INFO("[OccupancyGridMapMerge]: Starting printing values");     
    std::vector<int8_t> vals;
    unsigned width = map->info.width;
    unsigned height = map->info.height;
  
    ROS_INFO("[OccupancyGridMapMerge]: for cycle");     
    for(unsigned h = 0; h < height;++h){
      for(unsigned w = 0; w < width; ++w){
        /* print all values */
        bool is_in_vals = false;
        for(auto val : vals){
          if(map->data[h*height + w] == val){
            is_in_vals = true;
            break;
          }
        }
        if(!is_in_vals){
          vals.push_back( map->data[h*height + w]);
        }
      }
    }
  
    for(auto val : vals){
      ROS_INFO("%d", val);
    }
  }
  //}
  
  /* printValuesInImage() method //{ */
  void Merger2d::printValuesInImage(const sensor_msgs::Image& img){
    ROS_INFO("[OccupancyGridMapMerge]: Starting printing Image values");     
    std::vector<uint8_t> vals;
    unsigned width = img.width;
    unsigned height = img.height;
  
    ROS_INFO("[OccupancyGridMapMerge]: Width: %d  Height: %d", width, height);     
  
    for(unsigned h = 0; h < height;++h){
      for(unsigned w = 0; w < width; ++w){
        /* print all values */
        bool is_in_vals = false;
        for(auto val : vals){
          if(img.data[h*height + w] == val){
            is_in_vals = true;
            break;
          }
        }
        if(!is_in_vals){
          vals.push_back(img.data[h*height + w]);
        }
      }
    }
  
    for(auto val : vals){
      ROS_INFO("%d", val);
    }
  }
  //}
  
  /* rotateOccupancyGridByAngle() method //{ */
nav_msgs::OccupancyGridPtr Merger2d::transformOccupancyGridByPose(nav_msgs::OccupancyGridConstPtr og, geometry_msgs::PosePtr pose){
    cv_bridge::CvImagePtr img = convertOGToCvImg(og);
    transformCvImageByPose(img, pose, og->info.origin);
    return convertCvImgToOG(img, og);
  }
  //}

  //}
  
} //namespace merger2d

