#include "process_sensor_data.h"
#include "single_robot_mapping.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <ostream>
#include <string>

// namespace plt = matplotlibcpp;

std::unique_ptr<lidar_data> lidarData;
std::unique_ptr<robot_odometry> robotOdom;
std::unique_ptr<map_environment> robotMap;

constexpr const char *mainFilePath = __FILE__;

template <typename containerType>
std::vector<std::vector<containerType>>
readFile(const char *rootDataDirectory, const char *dataFile,
         std::vector<int> useFullFields) {
  std::string data_file =
      std::string(rootDataDirectory) + std::string(dataFile);
  std::cout << "Reading: " << data_file << std::endl;

  if (!std::filesystem::exists(data_file)) {
    std::cerr << "File Does Not Exist: " << data_file << std::endl;
    exit(EXIT_SUCCESS);
  }
  std::vector<std::vector<containerType>> data;
  std::string textData;
  std::ifstream dataPtr(data_file);
  while (getline(dataPtr, textData)) {
    std::istringstream iss(textData);
    std::vector<std::string> words;
    std::string word;
    while (iss >> word) {
      words.push_back(word);
    }

    std::vector<containerType> usefulData;
    usefulData.reserve(useFullFields.size());
    for (auto num : useFullFields) {
      usefulData.push_back(
          static_cast<containerType>(std::stod(words.at(num))));
    }
    data.push_back(usefulData);
  }
  return std::move(data);
}

int main(int argc, char **argv) {

  // Get the Data Path
  std::string path = mainFilePath;
  auto pos = path.find_last_of('/');
  if (pos != std::string::npos) {
    path = path.substr(0, pos);
  }
  pos = path.find_last_of('/');
  if (pos != std::string::npos) {
    path = path.substr(0, pos);
  }
  path.append("/data/");
  std::vector<std::vector<int>> motor_ticks = readFile<int>(
      path.c_str(), "Unit A/robot4_motors.txt", std::vector<int>{1, 2, 6});

  std::vector<int> time, leftVal, rightVal;
  //  for (auto tick : motor_ticks) {
  //    time.push_back(tick.at(0));
  //    leftVal.push_back(tick.at(1));
  //    rightVal.push_back(tick.at(2));
  //  }
  //  // Plot data
  //  plt::plot(time, leftVal);
  //  plt::plot(time, rightVal);
  //  plt::title("Motor Ticks");
  //  plt::xlabel("time");
  //  plt::ylabel("tick");
  //  plt::legend();
  //  plt::show();
  return 0;
}
