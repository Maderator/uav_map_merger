/**
 * @file transformations.h
 * @brief support functions and functions following compounding notation from apendix in <a href="https://ieeexplore.ieee.org/document/6776469">this paper</a>
 * @author Jan Maděra <maderja\fel.cvut.cz> (janmadera97\gmail.com)
 * @date 2020-06-17
 */

#pragma once
#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include "types.h"
#include <math.h>

// ros message types
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cv_bridge/cv_bridge.h>
 
// shared pointers
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

// for tf2 operations (fromMsg and toMsg) in getYaw and setYaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

// debugging
#include <ros/console.h>

/* FUNCTIONS FOLLOWING THE COMPOUNDING NOTATION in the apendix of "Multivehicle Cooperative 
 * Local Mapping: A Methodology Based on Occupancy Grid Map Merging", 
 * Hao Li et al., 2014     
 */

namespace merger2d { 

/* UAV FRAME OF REFERENCE POSE TRANSFORMATION //{ */

  /**
   * @brief Compute pose of received UAV relative to the pose of the current UAV in its map given the relative pose of two maps and UAV local poses
   *
   * Poses should be given in the meter units (as is default in ROS 
   * but not in our evolution process where we use "resolution" units)
   *
   * @param rel_map_p Relative pose of the two maps
   * @param loc_cur_p Relative pose of the current UAV in its map
   * @param loc_rec_p Relative pose of the other   UAV in the received map
   *
   * @return   
   */
  geometry_msgs::PosePtr recUAVRelToCurUAV(geometry_msgs::PoseConstPtr rel_map_p, geometry_msgs::PoseConstPtr loc_cur_p, geometry_msgs::PoseConstPtr loc_rec_p);

  /**
   * @brief Compute pose of UAV relative to the (0,0) point in the Global map given the local pose of the UAV and the pose of the UAV map relative to the Global map
   *
   * Poses should be given in the meter units (as is default in ROS 
   * but not in our evolution process where we use "resolution" units)
   *
   * WARNING: Do not use on UAV that has created the global map. Its pose is
   * already in the frame of reference of the global map
   *
   * @param rel_map_p Pose of the map of the UAV relative to the global map
   * @param loc_rec_p Local pose of the UAV
   *
   * @return   
   */
  geometry_msgs::PosePtr poseUAVLocalToGlobal(geometry_msgs::PoseConstPtr rel_map_p, geometry_msgs::PoseConstPtr loc_rec_p);

//}

/* COMPOUNDING FUNCTIONS //{ */

/** 
 * @brief Compound two 3D poses.
 *
 * Basically the new pose is on a circle with radius of distance between p2 and origin
 * and center in p1. The yaw of p1 determines where on the circle will the compounded
 * pose ends. With yaw = 0 rad the final pose ends right on the position where would
 * p2 be if we transformed whole space so that the p1 is in origin. With increasing
 * yaw the final pose moves counterclokwise on circle. 
 * New yaw will be p1's yaw + p2's yaw. 
 *
 * @param p1 Pose in the center of the circle.
 * @param p2 Pose giving the radius of the circle.
 *
 * @return Compounded pose on the circle boundary on yaw1 + yaw2 rad position.
 */
geometry_msgs::PosePtr compoundPose(const geometry_msgs::PoseConstPtr p1, const geometry_msgs::PoseConstPtr p2);
geometry_msgs::PosePtr compoundPose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
geometry_msgs::PosePtr compoundPose(const geometry_msgs::PoseConstPtr p1, const geometry_msgs::Pose& p2);

/** 
 * @brief Moves point the same way as in compoundPose but without given yaw (angle).
 *
 * @param pose Pose in the center of the circle.
 * @param point Point that is used to determine radius of circle.
 *
 * @return Transformed point on the circle by yaw of the pose.
 */
geometry_msgs::PointPtr transformPointXY(const geometry_msgs::PoseConstPtr pose, const geometry_msgs::PointConstPtr point);

/** 
 * @brief Moves point the same way as in compoundPose but without given yaw (angle).
 *
 * @param pose Pose in the center of the circle.
 * @param cell Point that is used to determine radius of circle.
 *
 * @return Transformed point on the circle by yaw of the pose.
 */
geometry_msgs::PointPtr transformPointXY(const geometry_msgs::PoseConstPtr pose, const Cell2DConstPtr cell);

/**
 * @brief Move pose on a circle around origin.
 *
 * Move pose on a circle with center in the origin and the given pose being
 * on the boundary of the circle. Yaw indicates where the resulting pose will be.
 * If the yaw is 0 rad then it will be on the oposite side of a circle then
 * the given pose. If the yaw is increasing then the pose will move clockwise
 * by angle from oposite side of the circle than original pose.
 *
 * @param pose Pose on the boundary of the circle.
 */
geometry_msgs::PosePtr invertPose(const geometry_msgs::PoseConstPtr pose);
geometry_msgs::PosePtr invertPose(const geometry_msgs::Pose& pose);

//}

// END OF COMPOUNDING NOTATION FROM PAPER APENDIX

// FUNCTIONS FOR TRANSFORATION OF IMG MAPS BY POSE
/* IMAGE TRANSOFRMATION FUNCTIONS //{ */

/**
 * @brief Transform occupancy grid map in form of image using given pose.
 *
 * Translate og image by pose's (x,y,z) coordinates vector and rotate it by
 * yaw angle of given pose.
 *
 * @param img_map Cv image to be transformed.
 * @param pose Pose describing translation and rotation of the img_map.
 * @param gui If true, show image before and after the transformation. This will stop the execution of the program. Press escape to continue.
 */
void transformCvImageByPose(cv_bridge::CvImagePtr img_map, const geometry_msgs::PoseConstPtr pose, const geometry_msgs::Pose& origin, bool gui = false);

/**
 * @brief Transform and copy occupancy grid map in form of image using given pose.
 *
 * Translate og image by pose's (x,y,z) coordinates vector and rotate it by
 * yaw angle of given pose.
 *
 * @param img_map Cv image to be transformed.
 * @param pose Pose describing translation and rotation of the img_map.
 * 
 * @return Transformed image as a transformed copy of the original parameter img_map
 */
cv_bridge::CvImagePtr copyAndTransformCvImageByPose(const cv_bridge::CvImagePtr img_map, const geometry_msgs::PoseConstPtr pose, const geometry_msgs::PoseConstPtr origin);

//}

/* OpenCV //{ */

/**
 * @brief rotates image around its center for given angle in radians
 *
 * @param src Source image to be rotated
 * @param angle Angle in radians. How much the image should be rotated
 */
void rotateAroundOrigin(cv::Mat src, double angle, const geometry_msgs::Pose& origin);

/**
 * @brief translates image by given points x and y coords
 *
 * @param src Source image to be translated
 * @param translation Point where x and y coords indicate the translation
 */
void translateImage(cv::Mat src, const geometry_msgs::Point& translation);

//}

/* YAW OPERATIONS //{ */

/**
 * @brief Get yaw angle from quaternion.
 *
 * @param q Quaternion from which the yaw is extracted.
 * @return Yaw angle.
 */
double getYaw(const geometry_msgs::Quaternion& q);

/**
 * @brief Set yaw angle of quaternion.
 *
 * @param q Quaternion in which the yaw is set.
 * @yaw_new Yaw angle to be set in Quaternion.
 */
void setYaw(const double& yaw_new, geometry_msgs::Quaternion& q);

//}

/* QUATERNION OPERATIOS //{ */

/**
 * @brief Set quaternion value
 *
 * @param roll Input roll
 * @param pitch Input pitch
 * @param yaw Input yaw
 * @param q Output quaternion
 */
void setRPY(const double& roll, const double& pitch, const double& yaw, geometry_msgs::Quaternion& q);

/**
 * @param q[in]
 * @param roll[out]
 * @param pitch[out]
 * @param yaw[out]
 */
void getRPY(geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw);

//}


/* remapVarToUnitInterval() //{ */

/**
 * @brief Linear remap of number from interval <a,b> to interval <0,1>
 *
 * @param x Value in old interval
 * @param min Lower boundary a of interval <a,b>
 * @param max Upper boundary b of interval <a,b>
 *
 * @return new value of x in remaped interval  
 */
float remapVarToUnitInterval(float x, float min, float max);

//}

} // namespace merger2d

#endif // TRANSFORMATIONS_H
