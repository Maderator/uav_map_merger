/**
 * @file verification.h
 * @brief  
 * @author Jan Maděra <maderja1@fel.cvut.cz> (janmadera97@gmail.com)
 * @date 2020-08-11
 */

#include "types.h"
#include <vector>
#include <utility>
#include <memory>
#include <random>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>

#include <iostream>
#include <fstream>

#ifndef VERIFICATION_H
#define VERIFICATION_H

namespace verification {

  /**
   * @brief Print data for graph of dependence of the fitness function and time of epoch  
   * // This only prints epoch by ROS_INFO_STREAM not writing to csv yet
   *
   * @param cum_time Cumulative time of epoch
   * @param fitness_func Fitness function value
   *
   * @return 0 if ok
   */
int epochToCSV(std::vector<double>& cum_time, std::vector<float> fitness_func);

/**
 * @brief  Init the header of the CSV file
 *
 * @param fname name of the CSV file
 *
 * @return 0 if the initialization was ok
 */
int initCSVMapFile(std::string fname);

/**
 * @brief add one record
 *
 * @param og Occupancy grid to be added
 * @param iter Number of iteration of the merging algorithm
 * @param time Time it took to do the iteration of map merging algorithm
 * @param fnam Name of the CSV file
 */
int addOGToCSVfile(int evolution_number, 
    double iter_time,
    OccupancyGridCellsPtr merged_map,
    unsigned map_consistency,
    double accept_index, 
    std::string fname);

/**
 * @brief Print occupancy grid map to a ofstream
 *
 * @param og Occupancy grid which we are recording
 * @param f An open stream to which we are writing
 */
int printMapCellsToOfstream(OccupancyGridCellsPtr ogc, std::ofstream& f);

} // namespace verification

#endif // VERIFICATION_H
