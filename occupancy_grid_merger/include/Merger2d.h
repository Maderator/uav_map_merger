/**
 * @file Merger2d.h
 * @author Jan Maděra <maderja@fel.cvut.cz> (janmadera97@gmail.com)
 * @version 0.0.1
 * @date 2020-06-02
 */

#pragma once
#ifndef MERGER2D_H
#define MERGER2D_H

/* INCLUDES //{ */

/* My optimalization */
#include "transformations.h"
#include "Evolution.h"
#include "Population.h"
#include "types.h"
#include "verification.h"

// for pi constant
#include <math.h>
#include <algorithm>

/* for smart pointers (do not use raw pointers) */
#include <memory>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <utility>
#include <vector>

/* for storing information about the state of the uav (occupancy grid map) */
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

/* ROS includes for data transformation between ROS and OpenCV */
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

/* for conversion betveen occupancy grid and image */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// namedWindow highgui.hpp
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// for tf2 operations (fromMsg and toMsg) in getYaw and setYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// debuging
#include <ros/console.h>

/* OpenCV includes for gui */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//}

namespace merger2d {

/**
 * Implementation of occupancy grid map merger 
 *
 * based on paper Multivehicle Cooperative Local Mapping: A Methodology Based on Occupancy 
 * Grid Map Merging by Hao Li, Manabu Tsukada, Fawzi Nashashibi, and Michel Parent
 * IEEE TRANSACTIONS ON INTELLIGENT TRANSPORTATION SYSTEMS, VOL. 15, NO. 5,OCTOBER 2014
 */
class Merger2d {

public:

  /* CONSTRUCTORS //{ */  

  Merger2d();

  /**
   * @brief Only local position of current drone is known
   *
   * When local position of other drone in its own map is unknown, we cannot approximate
   * the position of the other drone relative to the current drone. So this means that
   * the merger will only merge map, but it will not approximate the position of the
   * second drone.
   * 
   */
  Merger2d(
      nav_msgs::OccupancyGridConstPtr cur_map,
      nav_msgs::OccupancyGridConstPtr rec_map,
      geometry_msgs::PoseConstPtr loc_cur_p,
      EvolutionParams params
      );

  /**
   * @brief Both local position of current and other drone is known
   *
   * We can approximate position of other drone relative to this drone when we know
   * position of both drones in their maps
   */
  Merger2d(
      nav_msgs::OccupancyGridConstPtr cur_map,
      nav_msgs::OccupancyGridConstPtr rec_map,
      geometry_msgs::PoseConstPtr loc_cur_p,
      geometry_msgs::PoseConstPtr loc_rec_p,
      EvolutionParams params);

  /**
   * @brief Global positions are also known (for example from gps).
   *
   * This will be hardly ever used because this is implemented for indoor use
   */
  Merger2d(
      nav_msgs::OccupancyGridConstPtr cur_map, 
      nav_msgs::OccupancyGridConstPtr rec_map,
      geometry_msgs::PoseConstPtr loc_cur_p,
      geometry_msgs::PoseConstPtr loc_rec_p,
      geometry_msgs::PoseConstPtr glob_cur_p,
      geometry_msgs::PoseConstPtr glob_rec_p,
      EvolutionParams params);
 
  //}

  /* GETTERS AND SETTERS //{ */  
 
  /// maps are stored in CvImagePtr and it is useless to store them in OG
  cv_bridge::CvImagePtr getCurrentCvMap() const;
  void setCurrentMap(nav_msgs::OccupancyGridConstPtr new_c_map);
  
  /// maps are stored in CvImagePtr and it is useless to store them in OG
  cv_bridge::CvImagePtr getReceivedCvMap() const;
  void setReceivedMap(nav_msgs::OccupancyGridConstPtr new_r_map);

  float getAcceptanceIndex() const;
  
  /// get pose of other UAV map relative to the current UAV map
  geometry_msgs::PoseConstPtr     getRelativeMapPose() const;

  geometry_msgs::PoseConstPtr     getLocalCurrentPosition() const;
  void setLocalCurrentPosition(geometry_msgs::PoseConstPtr new_lcp);

  geometry_msgs::PoseConstPtr     getGlobalCurrentPosition() const;
  void setGlobalCurrentPosition(geometry_msgs::PoseConstPtr new_gcp);
  
  geometry_msgs::PoseConstPtr     getLocalRecievedPosition() const;
  void setLocalRecievedPosition(geometry_msgs::PoseConstPtr new_lrp);
  
  geometry_msgs::PoseConstPtr     getGlobalRecievedPosition() const;
  void setGlobalRecievedPosition(geometry_msgs::PoseConstPtr new_grp);

  EvolutionParams                 getEvolutionparams() const;
  void setEvolutionParams(EvolutionParams params);
  
  //}


  void initializeEvolver();
  void initializeGAGeneration();

  /**
   * @brief Merge maps with relative pose being the best pose of given generation. The generation is also evolved for a short period of time.
   *
   * @return Global occupancy grid map created by merging the local OG maps of two UAVs
   */
  nav_msgs::OccupancyGridPtr mergeMapsContinuously();
  
  /* findOptimalRelativePose() method //{ */
 
 /**
   * @brief  Find the optimal relative pose that optimizes the consistency measure between two OG maps
   *
   * Optimal relative pose labeled \f$ p_{BA} \f$ optimizes consistency measure between
   * map \f$ A \f$ and \f$ p_{BA} \oplus B \f$ map. The \f$ \oplus \f$ is compounding
	 * operation of pose and occupancy grid. Optimal pose \f$ p_{BA} \f$ is computed like
   * this:
   * \f$ pBA = argmax_{pBA} F_c(A, p_{BA} \oplus B) \f$ 
   *
   * @param prev_merg_info The result of previous merging epoch
   *
   * @return Optimal relative pose  
   */
  geometry_msgs::PoseConstPtr findOptimalRelativePose(TimerMergingInfo& prev_merg_info);

  /**
  * @param new_rec_map New received map to be used in optimalization
  */
  geometry_msgs::PoseConstPtr findOptimalRelativePose(nav_msgs::OccupancyGridConstPtr new_rec_map);

	/**
	* @param new_cur_map New map of the current uav to be used in optimalizatino
	* @param new_rec_map New received map to be used
	*/
  geometry_msgs::PoseConstPtr findOptimalRelativePose(
      nav_msgs::OccupancyGridConstPtr new_cur_map,
      nav_msgs::OccupancyGridConstPtr new_rec_map);

  //}

  /* findOccupiedCells() method //{ */
  /**
   * @brief In occupancy grid find cells with maximum occupancy state
   *
   * @param og Occupancy grid in which we want to find the cells
   *
   * @return Found cells
   */
  OccupancyGridCellsPtr findOccupiedCells(nav_msgs::OccupancyGridConstPtr og);
  //}

  /* mergeCells() method //{ */
  /**
   * @brief merge the occupied cells of both maps into one
   *
   * This is mainly used for the experimental part of the thesis
   *
   * @param pose Relative pose of the two maps
   *
   * @return Merged occupied cells
   */
  OccupancyGridCellsPtr mergeCells(geometry_msgs::PoseConstPtr pose);
  //}

/**
 * @brief  
 *
 * @param og Occupancy grid to be transformed
 * @param angle Angle in radians by which the occupancy grid will be transformed
 *
 * @return Rotated occupancy grid
 */
nav_msgs::OccupancyGridPtr transformOccupancyGridByPose(nav_msgs::OccupancyGridConstPtr og, geometry_msgs::PosePtr pose);
 
  /* setMemberVars() method //{ */

  /**
   * @brief Make initialization of member variables that are same for every constructor
   */
  void setMemberVars(nav_msgs::OccupancyGridConstPtr cur_map, nav_msgs::OccupancyGridConstPtr rec_map);
  //}
  
  /**
   * @brief Set the local poses of current uav and the received UAV
   *
   * @param cur_loc_p Local pose of current UAV
   * @param rec_loc_p Local pose of second UAV from which we received data
   */
  void setLocalPoses(geometry_msgs::PoseConstPtr cur_loc_p, geometry_msgs::PoseConstPtr rec_loc_p);

private:  
  
  /* MEMBER VARIABLES //{ */ 

  /// Relative pose with higest heuristic value
  geometry_msgs::PoseConstPtr rel_map_pose_;
 
  /// Acceptance index computed as is described in the bachelor thesis text
  float accept_index_;

  /// converted map to CvImage owned by current uav
  cv_bridge::CvImagePtr cv_cur_;
  
  /// Current drone occupancy grid
  nav_msgs::OccupancyGridConstPtr og_cur_;

  /// Local maximum occupied cells
  OccupancyGridCellsPtr occupied_cells_cur_;

  /// converted map to CvImage owned recieved from other uav
  cv_bridge::CvImagePtr cv_rec_;

  /// Local maximum occupied cells
  OccupancyGridCellsPtr occupied_cells_rec_;
  
  /// Received drone occupancy grid
  nav_msgs::OccupancyGridConstPtr og_rec_;
  
  /// position of uav inside current map (local current pose)
  geometry_msgs::PosePtr loc_cur_p_;
  
  /// estimated position of uav in the world (global current pose)
  geometry_msgs::PosePtr glob_cur_p_;
  
  /// position of other uav inside recieved map (local recieved pose)
  geometry_msgs::PosePtr loc_rec_p_;
  
  /// estimated position of other uav in the world (global recieved pose)
  geometry_msgs::PosePtr glob_rec_p_;

  /// Evolution interface used for evolution of generation
  evolution::Evolution evolver_;

  /// Generation of relative poses optimized by evolution
  evolution::GenerationPtr evolution_generation_;

  bool has_loc_r_;
  bool has_glob_c_;
  bool has_glob_r_;

  /// error of rotation around z axis
  float orientation_error_range_;

  /// range on axis with highest error range
  float position_error_range_;

  /// parameters needed for evolution. Contains occupancy and acceptance threshold, population size and gui flag
  EvolutionParams params_;
 
  //}

  /* computeacceptanceIndex() method //{ */
  /**
   * @brief Computes acceptance index as is defined in the bachelor thesis
   *
   * @param agree Number of cells which have same value in both maps
   * @param disagree Number of cells which have different value in both maps
   */
  void computeAcceptanceIndex(long unsigned agree, long unsigned disagree);
  //}

  /* convertPoseUnitsFromMetersToResolution() method //{ */
  /**
   * @brief This method converts all 4 poses from meters to pixels units.
   * 
   * Position and orientation are given in meters but everything else is computed in
   * terms of size of pixels in images and cells in occupancy grid. This method converts
   * all 4 poses to the same units as everything else. Current occupancy grid resolution 
   * is used to compute new poses.
   */
  void convertPosesUnitsFromMetersToResolution();
  //}
  
  /* convertPosesAndPrevPosesUnitsFromMetersToResolution() method //{ */
  /**
   * @brief Converts poses from previous merging and all 4 poses from meters to pixel units
   *
   * @param prev_rel_pose relative pose of maps from previous map merging
   * @param prev_loc_cur_p local pose of current UAV from previous map merging
   * @param prev_loc_rec_p received local pose of UAV from previous map merging
   */
  void convertPosesAndPrevPosesUnitsFromMetersToResolution(geometry_msgs::PosePtr prev_rel_pose, geometry_msgs::PosePtr prev_loc_cur_p, geometry_msgs::PosePtr prev_loc_rec_p);
  //}

  /* convertPoseUnitsToRes() method //{ */
  /**
   * @brief Converts pose position from meters to cells units
   *
   * @param pose Pose with position given in meters that will be converted to "cells" units
   */
  void convertPoseUnitsToRes(geometry_msgs::PosePtr pose, nav_msgs::OccupancyGridConstPtr og);
  //}
  
  /* convertCurPoseUnitsFromResToMeters() method //{ */
  /**
   * @brief Converts current uav's pose to units of meters from cells units
   *
   * @param pose Pose with position given in "cells" units that will be converted to meters
   */
  void convertPoseUnitsFromResToMeters(geometry_msgs::PosePtr pose, nav_msgs::OccupancyGridConstPtr og);
  geometry_msgs::PosePtr convertPoseUnitsFromResToMeters(const geometry_msgs::PoseConstPtr pose, const nav_msgs::OccupancyGridConstPtr og) const;
  //}
 
  /* convertPoseUnitsFromResToMetersWithZeroOriginNew() method //{ */
  geometry_msgs::PosePtr convertPoseUnitsFromResToMetersWithZeroOriginNew(const geometry_msgs::PoseConstPtr pose, float res) const;
  //}
  
  /* convertRecPoseUnitsToRes() method //{ */
  /**
   * @brief Converts pose of other uav to cells units for given pose
   *
   * @param pose Pose with position given in meters that will be converted to "cells" units
   */
  void convertRecPoseUnitsToRes(geometry_msgs::PosePtr pose);
  //}
  
  /* SUPPORT METHODS for blendMaps() method//{ */
  
  /**
   * @brief Blends images of maps with same width and height
   *
   * @param img1 First map in form of image to be blended
   * @param img2 Second map in form of image to be blended
   * @param unknown_thr threshold for cell state being unknown
   */
  void blendSameSizeImgs(cv::Mat img1, cv::Mat img2, bool unknown_thr);
  
  /**
   * @brief Blends images of maps with different width and height
   *
   * @param img1 First map in form of image to be blended
   * @param img2 Second map in form of image to be blended
   * @param unknown_thr threshold for cell state being unknown
   */
  void blendDifferentSizeImgs(cv::Mat img1, cv::Mat img2, bool unknown_thr);
  
  //}

  /* blendMaps() method //{ */
  /**
   * @brief Takes two images of maps and blend them together
   *
   * -Each "occupied" pixel (value of pixel > occupancy_threshold) stays occupied
   * -If at least one map has "unoccupied" pixel and none is "occupied" then this pixel
   *  will become "unoccupied"
   * -Each "occupied" pixel (value of pixel > occupancy_threshold) stays occupied
   * -If at least one map has "unoccupied" pixel and none is "occupied" then this pixel
   *  will become "unoccupied"
   * -If both of the pixels values are near zero, then  resulting pixel will be marked as "unknown"
   * -This method can blend two maps of different width and height but it is better to 
   *  blend maps with same size of dimensions
   *
   * @param map1 image of first map
   * @param map2 image of second map
   *
   * @return blended map:
   */
  cv_bridge::CvImagePtr blendMaps(cv_bridge::CvImagePtr map1, cv_bridge::CvImagePtr map2);
  //}

  /* CONVERSION OG<->IMG //{ */
     
  /**
   * @brief Converts Occupancy Grid to *Cv* Image
   *
   * @param map OccupancyGrid to be converted
   *
   * @return Converted *Cv* Image 
   */
  cv_bridge::CvImagePtr convertOGToCvImg(nav_msgs::OccupancyGridConstPtr map);
  
  /**
   * @brief Converts Occupancy Grid to *Cv* Image
   *
   * @param map OccupancyGrid to be converted
   * @param cols Wanted width of converted cv image
   * @param rows Wanted height of converted cv image 
   *
   * @return Converted *Cv* Image 
   */
  cv_bridge::CvImagePtr convertOGToCvImg(nav_msgs::OccupancyGridConstPtr map, unsigned cols, unsigned rows);
 
  /**
   * @brief Converts received Occupancy Grid to *Cv* Image with same or larger 
   * dimensions than occupancy grid of current map
   *
   * @param map Received occupancy grid to be converted
   * @param cols width of current occupancy grid map cv image
   * @param rows height of current occupancy grid map cv image
   *
   * @return Converted *Cv* Image 
   */
  cv_bridge::CvImagePtr convertRecOGToCvImg(nav_msgs::OccupancyGridConstPtr rec_map, unsigned cur_cols, unsigned cur_rows);
  
  /**
   * @brief Converts Occupancy Grid to Image
   *
   * @param map OccupancyGrid to be converted
   *
   * @return Converted Image 
   */
  sensor_msgs::ImagePtr convertOGToImg(const nav_msgs::OccupancyGridConstPtr map);
  
  /**
   * @brief Converts Occupancy Grid to Image of same or LARGER size than given map
   *
   * Based on parameters cols and rows the image will be of same size as given map or 
   * larger. If the cols or rows parameters are smaller than size of given map, this
   * will throw an error info message and will result in undefined behaviour
   *
   * @param map OccupancyGrid to be converted
   * @param cols Width of created image
   * @param rows Height of created image
   *
   * @return Converted Image 
   */
  sensor_msgs::ImagePtr convertOGToImg(const nav_msgs::OccupancyGridConstPtr map, unsigned cols, unsigned rows);
  
  /** 
   * @brief Converts CV image to a pointer to occupancy grid.
   *
   * @param cv_img_p Pointer to an CV image that is to be converted.
   *
   * @return image converted from CV image
   */
  sensor_msgs::ImagePtr convertCvImgToImg(
      const cv_bridge::CvImageConstPtr cv_img_p);

  /** 
   * @brief Convert image to a pointer to occupancy grid using occupancy threshold parameter
   *
   * @param img_p Pointer to a CV image that is to be converted.
   * @parama cur_og Pointer to occupancy grid of current uav.
   *
   * @return occupancy grid converted from image
   */
  nav_msgs::OccupancyGridPtr convertCvImgToOG(
      const cv_bridge::CvImageConstPtr img_p, 
      const nav_msgs::OccupancyGridConstPtr cur_og);
  
  //}

  /* ERROR METHODS //{ */
  
  /**
   * @brief Set error ranges for orientation and position according to acceptance index of previous map merging
   *
   * If the previous map merging was successful (It had acceptance index well above 0.9) this will set small error ranges. If it was unsuccessful, this will set large error ranges.
   *
   * @param prev_accept_index Acceptance index of previous map merging
   */
  void computeErrorRanges(float prev_accept_index);
  
  /** 
   * @brief sets orientationErrorRange and positionErrorRange
   *
   * Used to initialize population with noise of orientation error in range 
   * [-orientation_err, +orientation_err] and position error in range 
   * [-position_err, +position_err]
   *
   * @param orientation_err Error of orientation (max Pi)
   * @param positionErr Error of position value 
   */
  void setErrorRanges(float orientation_err, float position_err);
  
  //}
 
  /* SUPPORT FUNCTIONS //{ */
 
  /**
   * @brief Support function for findOptimalRelativePose method
   *
   * Sets unrecieved poses to value of local pose of this uav in its map
   */
  void initializeUnreceivedPoses();

  /**
   * @brief  Initializes gui for cv images if parameter gui is true
   *
   * @param gui No GUI will be generated if false
   */
  void initGUI(bool gui);

  //}

  /* DEBUG FUNCTIONS //{ */
 
  /**
   * @brief  Prints occupied cells positions in occupancy grid map
   *
   * @param cells Coordinates of occupied cells
   */
  void printOccupiedCells(OccupancyGridCellsPtr cells);
  
  /**
   * @brief Prints whole occupancy grid map
   *
   * @param map Occupancy grid map to be printed
   */
  void printValuesInOccupancyGridMap(nav_msgs::OccupancyGridConstPtr map);
  
  /**
   * @brief Prints whole image in numbers 
   *
   * @param img Image to be printed
   */
  void printValuesInImage(const sensor_msgs::Image& img);
  
//}

};

} // namespace merger2d

#endif // MERGER2D_H
