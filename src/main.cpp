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

template<typename T>
void plot_graph(
  std::vector<T> &x,
  std::vector<T> &y,
  const char *chartTitle,
  const char *xLabel,
  const char *yLabel
) {
  // Plot the pose data
  plt::figure_size(800, 800);
  plt::plot(x, y, "b-o");
  plt::xlabel(xLabel);
  plt::ylabel(yLabel);
  plt::title(chartTitle);
  plt::grid(true);
  plt::show();
}

std::ostream &operator<<(std::ostream &os, const pose_t &pose) {
  os << "X: " << pose.x << " | Y: " << pose.y << " | Theta: " << pose.theta;
  return os;
}

std::vector<float> x_data, y_data, theta_data;
void step(int ts, const encoder_ticks_t encoderTick) {
  robotOdom->m_update_pose(encoderTick);
  pose_t current_pose = robotOdom->get_current_pose();
  x_data.push_back(current_pose.x);
  y_data.push_back(current_pose.y);
  theta_data.push_back(current_pose.theta);
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
  std::vector<std::vector<int> > motor_ticks = readFile<int>(
    path.c_str(), "Unit A/robot4_motors.txt", std::vector<int>{1, 2, 6});

  auto startingTick = motor_ticks.at(0);
  encoder_ticks_t startingVal{
    .left = static_cast<float>(startingTick.at(1)),
    .right = static_cast<float>(startingTick.at(2))
  };
  robotOdom = std::make_unique<robot_odometry>(
    0.349F,
    150.0F,
    pose_t{0.0F, 0.0F, 0.0F},
    startingVal);

  for (std::size_t i = 0; i < motor_ticks.size(); i++) {
    auto encoderReading = motor_ticks.at(i);

    int timeStep = encoderReading.at(0);
    encoder_ticks_t encoderVal{
      .left = static_cast<float>(encoderReading.at(1)),
      .right = static_cast<float>(encoderReading.at(2))
    };
    step(timeStep, encoderVal);
  }
plot_graph(x_data,
  y_data,
  "Robot Pose",
  "x (mm)",
  "y (mm)");

  return 0;
}
