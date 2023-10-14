#include "verification.h"

namespace verification {

int epochToCSV(std::vector<double>& cum_time, std::vector<float> fitness_func) {
  ROS_INFO("iteration, fit, cumTime");
  for(size_t i = 0; i < cum_time.size(); ++i){
    ROS_INFO_STREAM(i+1 << ", " << fitness_func.at(i) << ", " << cum_time.at(i));
  }

  return 0;
}

int initCSVMapFile(std::string fname) {
  std::ofstream file;
  file.open(fname);
  file << "iteration, itertime, mapcons, ai\n";
  file.close();
  return 0;
}

int addOGToCSVfile(int evolution_number, 
    double iter_time,
    OccupancyGridCellsPtr merged_map,
    unsigned map_consistency,
    double accept_index, 
    std::string fname) {
  std::ofstream file;
  file.open(fname, std::ios_base::app); // append
  file << evolution_number << ", " << iter_time << ", " << map_consistency << ", " << accept_index << "\n";
  file.close();
  ROS_INFO("Written to file");
 // std::string suf = "_cells";
 // suf.append(std::to_string(evolution_number));
 // fname.append(suf);
 // file.open(fname);
  //printMapCellsToOfstream(merged_map, file);
  return 0;
}

int printMapCellsToOfstream(OccupancyGridCellsPtr ogc, std::ofstream& f) {
  f << "x, y\n";
  for(size_t i = 0; i+1 < ogc->size(); ++i){
    f << ogc->at(i)->col << ", " << ogc->at(i)->col << "\n";
  }
  f << ogc->at(ogc->size()-1)->col << ", " << ogc->at(ogc->size()-1)->row;
  return 0;
}

} // namespace verification
