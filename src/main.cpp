#include "process_sensor_data.h"
#include "single_robot_mapping.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <ostream>
#include <string>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

std::unique_ptr<lidar_data> lidarData;
std::unique_ptr<robot_odometry> robotOdom;
std::unique_ptr<map_environment> robotMap;

constexpr const char *mainFilePath = __FILE__;

template<typename containerType>
std::vector<std::vector<containerType> > readFile(
  const char *rootDataDirectory,
  const char *dataFile,
  std::vector<int> useFullFields) {
  std::string data_file =
      std::string(rootDataDirectory) + std::string(dataFile);
  std::cout << "Reading: " << data_file << std::endl;

  if (!std::filesystem::exists(data_file)) {
    std::cerr << "File Does Not Exist: " << data_file << std::endl;
    exit(EXIT_SUCCESS);
  }
  std::vector<std::vector<containerType> > data;
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
    for (auto num: useFullFields) {
      usefulData.push_back(
        static_cast<containerType>(std::stod(words.at(num))));
    }
    data.push_back(usefulData);
  }
  return std::move(data);
}

std::ostream &operator<<(std::ostream &os, const pose_t &pose) {
  os << pose.x << "\t" << pose.y << "\t" << pose.theta;
  return os;
}

std::vector<float> x_data, y_data, theta_data;
std::vector<pose_t> all_pose;

void step(int ts, const encoder_ticks_t encoderTick) {
  robotOdom->m_update_pose(encoderTick);
  pose_t current_pose = robotOdom->get_current_pose();
  x_data.push_back(current_pose.x);
  y_data.push_back(current_pose.y);
  theta_data.push_back(current_pose.theta);
  all_pose.push_back(current_pose);
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

  //Read Motor Ticks
  std::vector<std::vector<int> > motor_ticks = readFile<int>(
    path.c_str(),
    "/data/Unit A/robot4_motors.txt",
    std::vector<int>{1, 2, 6});
  //Read True Robot Position
  std::vector<std::vector<float> > true_robot_position = readFile<float>(
    path.c_str(),
    "/data/Unit A/robot4_reference.txt",
    std::vector<int>{2, 3});
  std::vector<float> x_true, y_true, theta_true;
  for (auto true_data: true_robot_position) {
    x_true.push_back(true_data.at(0));
    y_true.push_back(true_data.at(1));
  }
  //Read True Robot Position
  std::vector<std::vector<int> > robot_lidar_data = readFile<int>(
    path.c_str(),
    "/data/Unit A/robot4_scan.txt",
    std::vector<int>{2, 3});


  //Initialize Robot Odometry
  robotOdom = std::make_unique<robot_odometry>(
    0.349F,
    173.0F,
    pose_t{
      1850.0F,
      1897.0F,
      (213.0F / 180.0F) * M_PI
    },
    encoder_ticks_t{
      static_cast<float>(motor_ticks.at(0).at(1)),
      static_cast<float>(motor_ticks.at(0).at(2))
    },
    pose_t{
      0.0F,
      0.0F,
      0.0F
    });

  //Step The SLAM
  for (std::size_t i = 0; i < motor_ticks.size(); i++) {
    auto encoderReading = motor_ticks.at(i);

    int timeStep = encoderReading.at(0);
    encoder_ticks_t encoderVal{
      .left = static_cast<float>(encoderReading.at(1)),
      .right = static_cast<float>(encoderReading.at(2))
    };
    step(timeStep, encoderVal);
  }

  //Save Motor Position to File
  std::string pose_data = path;
  pose_data.append("/results/pose_data.txt");
  std::ofstream pose_data_file(pose_data, std::ios::app);
  if (!pose_data_file.is_open()) {
    std::cerr << "Failed to open file for writing." << std::endl;
    return EXIT_FAILURE;
  }
  for (auto current_pose: all_pose) {
    pose_data_file << "F\t" << current_pose << "\n";
    // std::cout << current_pose << std::endl;
  }
  pose_data_file.close();

  //Plot Graph
  plt::figure_size(800, 800);
  plt::plot(x_data, y_data, "b-o");
  plt::plot(x_true, y_true, "r-o");
  plt::xlabel("x (mm)");
  plt::ylabel("y (mm)");
  plt::title("Robot Pose");
  plt::grid(true);
  plt::show();

  return 0;
}
