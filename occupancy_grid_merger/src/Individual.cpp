/**  
 * @file
 * @author Jan Maděra <maderja\fel.cvut.cz> (janmadera97\gmail.com)
 * @date 2020-06-16
 */

#include "Individual.h"

namespace merger2d {

namespace evolution {
 
  Individual::Individual(geometry_msgs::PosePtr pose, OccupancyGridCellsPtr cells_rec, float occupancy_threshold, unsigned cur_map_width, unsigned cur_map_height, cv_bridge::CvImageConstPtr img_map_cur) : 
    pose_(pose), cells_rec_(cells_rec), occupancy_threshold_(occupancy_threshold), cur_map_width_(cur_map_width), cur_map_height_(cur_map_height), img_map_cur_(img_map_cur) {
      map_consistency_ = computeMapConsistency();
    }

  float Individual::getMapConsistency() const { return map_consistency_; }

  void Individual::setPose(geometry_msgs::PosePtr pose) { 
    pose_ = pose;
    // compute map consistency with new pose
    map_consistency_ = computeMapConsistency();
  }

  void Individual::setPose(geometry_msgs::PoseConstPtr pose) { 
    *pose_ = *pose; 
    // compute map consistency with new pose
    map_consistency_ = computeMapConsistency();
  }

  void Individual::setPose(geometry_msgs::PosePtr pose, float consistency) {
    *pose_ = *pose;
    map_consistency_ = consistency;
  }

  geometry_msgs::PoseConstPtr Individual::getPose() const { return pose_; }
  
  /* OBJECTIVE FUNCTION //{ */
  
  /* computeMapConsistency() method //{ */
  float Individual::computeMapConsistency() const
  {
    return computeMapConsistencyForGivenPose(pose_);
  }

  float Individual::computeMapConsistencyForGivenPose(geometry_msgs::PosePtr pose) const {
    float consistency = 0; // sum in which the final value will be stored
  
    geometry_msgs::PointPtr transformed_cell;
    for(auto cell : *cells_rec_){
      transformed_cell = transformPointXY(pose , cell);
      if(isPointInsideMap(transformed_cell, cur_map_width_, cur_map_height_)){
        float occupancyState = 
          static_cast<float>(
            img_map_cur_->image.data[static_cast<int>(transformed_cell->y)*cur_map_width_ +
                                     static_cast<int>(transformed_cell->x)]);  
        if(occupancyState > occupancy_threshold_){
          // dividing by image max value so that the occupancyState value is in interval <0,1>
          //consistency += occupancyState / IMG_MAX_OCCUP_VAL;
          consistency += 1;
        }
      }
    }
    return consistency;
  }
  //}
  
  /* isPointInsideMap() method //{ */
  bool Individual::isPointInsideMap(geometry_msgs::PointConstPtr p, unsigned width, unsigned height) const {
      if(p->x >= width || p->x < 0 || p->y >= height || p->y < 0){
        return false;
      }
      return true;
  }
  //}
  //}

} // namespace evolution

} // namespace merger2d
