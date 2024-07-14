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
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;
cv::Scalar red(0, 0, 255);
cv::Scalar green(0, 255, 0);
cv::Scalar blue(255, 0, 0);
cv::Scalar white(255, 255, 255);

std::unique_ptr<lidar_data> lidarObject;
std::unique_ptr<robot_odometry> robotOdomObject;
std::unique_ptr<map_environment> robotMap;

constexpr const char *mainFilePath = __FILE__;

template<typename containerType>
std::vector<std::vector<containerType> > readFile(
  const char *rootDataDirectory,
  const char *dataFile,
  std::vector<int> useFullFields) {
  std::string data_file = std::string(rootDataDirectory) + std::string(dataFile);
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
    if (!useFullFields.empty()) {
      usefulData.reserve(useFullFields.size());
      for (auto num: useFullFields) {
        usefulData.push_back(static_cast<containerType>(std::stod(words.at(num))));
      }
    } else {
      for (std::size_t idx = 3; idx < words.size(); idx++) {
        usefulData.push_back(static_cast<containerType>(std::stod(words.at(idx))));
      }
    }
    data.push_back(usefulData);
  }
  data.resize(data.size());
  return std::move(data);
}

std::ostream &operator<<(std::ostream &os, const pose_t &pose) {
  os << pose.x << "\t" << pose.y << "\t" << pose.theta;
  return os;
}

template<typename T, typename... Vectors>
int computeImageDimensions(const std::vector<T> &first, const Vectors &... others) {
  auto minmax_x = std::minmax_element(first.begin(), first.end());
  T min_x = *minmax_x.first;
  T max_x = *minmax_x.second;
  ([&](const std::vector<T> &vec) {
    auto minmax = std::minmax_element(vec.begin(), vec.end());
    min_x = std::min(min_x, *minmax.first);
    max_x = std::max(max_x, *minmax.second);
  }(others), ...);
  return static_cast<int>(max_x - min_x + 100);
}

template<typename T>
void plotGraphPoints(cv::Mat &image, const std::vector<T> &x, const std::vector<T> &y, const cv::Scalar &color,
                     int height) {
  // Draw points
  for (size_t i = 0; i < x.size(); ++i) {
    int xCoord = static_cast<int>(x[i]);
    int yCoord = height - static_cast<int>(y[i]);
    cv::circle(image, cv::Point(xCoord, yCoord), 2, color, cv::FILLED);
  }
}

template<typename T>
void plotLineGraph(cv::Mat &image, const std::vector<T> &x, const std::vector<T> &y, const cv::Scalar &color,
                   int height) {
  // Draw lines
  for (size_t i = 1; i < x.size(); ++i) {
    int xCoord1 = static_cast<int>(x[i - 1]);
    int yCoord1 = height - static_cast<int>(y[i - 1]);
    int xCoord2 = static_cast<int>(x[i]);
    int yCoord2 = height - static_cast<int>(y[i]);
    cv::line(image, cv::Point(xCoord1, yCoord1), cv::Point(xCoord2, yCoord2), color, 2);
  }
}

void drawLegend(cv::Mat &image, const std::vector<std::string> &labels, const std::vector<cv::Scalar> &colors,
                int x = 50, int y = 50) {
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.5;
  int thickness = 1;

  for (size_t i = 0; i < labels.size(); ++i) {
    cv::putText(image, labels[i], cv::Point(x + 20, y + i * 20), fontFace, fontScale, white, thickness);
    cv::rectangle(image, cv::Point(x, y + i * 20 - 10), cv::Point(x + 10, y + i * 20), colors[i], cv::FILLED);
  }
}

std::vector<std::vector<float> > lidar_derivatives;
std::vector<std::vector<coordinate_t> > lidar_coordinates;
std::vector<std::vector<obstacle_location_t> > obstacle_lists;
std::vector<std::vector<coordinate_t> > obstacle_coordinates;
std::vector<pose_t> all_pose;

void step(int ts, const encoder_ticks_t encoderTick, const std::vector<float> &lidar_data) {
  robotOdomObject->m_update_pose(encoderTick);
  const pose_t current_pose = robotOdomObject->get_current_pose();

  const auto derivative = lidarObject->calculate_derivative(lidar_data);
  const auto obstacles = lidarObject->find_obstacles(lidar_data, derivative);
  const auto lidar_map = lidarObject->convert_scan_to_coordinate(lidar_data, current_pose);
  const auto obstacle_coordinate = lidarObject->convert_obstacle_to_coordinate(obstacles, current_pose);

  all_pose.push_back(current_pose);
  lidar_derivatives.push_back(derivative);
  obstacle_lists.push_back(obstacles);
  lidar_coordinates.push_back(lidar_map);
  obstacle_coordinates.push_back(obstacle_coordinate);
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

  fs::path results_dir = path + "/results";

  // Delete the directory if it exists
  if (fs::exists(results_dir)) {
    fs::remove_all(results_dir);
    std::cout << "Deleted directory: " << results_dir << std::endl;
  }

  // Create the directory
  if (fs::create_directory(results_dir)) {
    std::cout << "Created directory: " << results_dir << std::endl;
  } else {
    std::cerr << "Failed to create directory: " << results_dir << std::endl;
  }

  /////////////////////////// Read Data ///////////////////////////////////////
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
  std::vector<std::vector<float> > robot_lidar_data = readFile<float>(
    path.c_str(),
    "/data/Unit A/robot4_scan.txt",
    std::vector<int>{});

  /////////////////////////// Initialize Objects ///////////////////////////////////////
  //Initialize Robot Odometry
  robotOdomObject = std::make_unique<robot_odometry>(
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

  //Lidar Scanning
  lidarObject = std::make_unique<lidar_data>(
    robot_lidar_data.at(0).size(),
    value_range_t{20.0F, std::numeric_limits<float>::max()},
    value_range_t{-2.09466781009F, 1.95504146993F},
    100.0F
  );

  /////////////////////////// Run SLAM ///////////////////////////////////////
  for (std::size_t i = 0; i < motor_ticks.size(); i++) {
    auto encoderReading = motor_ticks.at(i);

    int timeStep = encoderReading.at(0);
    encoder_ticks_t encoderVal{
      .left = static_cast<float>(encoderReading.at(1)),
      .right = static_cast<float>(encoderReading.at(2))
    };
    step(timeStep, encoderVal, robot_lidar_data.at(i));
  }

  /////////////////////////// Save Data ///////////////////////////////////////
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
  }
  pose_data_file.close();

  //Save Obstacles to File
  std::string obstacle_data = path;
  obstacle_data.append("/results/cylinders.txt");
  std::ofstream obs_data_file(obstacle_data, std::ios::app);
  if (!obs_data_file.is_open()) {
    std::cerr << "Failed to open file for writing." << std::endl;
    return EXIT_FAILURE;
  }
  for (const auto &obstacles: obstacle_coordinates) {
    obs_data_file << "D C";
    for (auto obstacle: obstacles) {
      obs_data_file << "\t" << obstacle.x << "\t" << obstacle.y;
    }
    obs_data_file << "\n";
  }
  obs_data_file.close();

  /////////////////////////// Plot Graph ///////////////////////////////////////
  {
    //Robot Pose
    std::vector<float> x_data, y_data, theta_data;
    for (auto [x,y,theta]: all_pose) {
      x_data.push_back(x);
      y_data.push_back(y);
      theta_data.push_back(theta);
    }

    auto width = computeImageDimensions(x_data, x_true);
    auto height = computeImageDimensions(y_data, y_true);

    std::vector<std::string> labels = {"Calculated Pose", "True Pose"};
    std::vector<cv::Scalar> colors = {red, green};
    cv::Mat pose_image = cv::Mat::zeros(height, width, CV_8UC3);

    plotGraphPoints(pose_image, x_data, y_data, red, height);
    plotGraphPoints(pose_image, x_true, y_true, green, height);
    drawLegend(pose_image, labels, colors);
    cv::imshow("Robot Pose", pose_image);
  }

  //Lidar Raw Data
  int idx = 0; {
    std::vector<float> x_axis, lidar_data, lidar_derivative;
    for (size_t i = 0; i < robot_lidar_data.at(idx).size(); i++) {
      x_axis.push_back(i);
      lidar_data.push_back(robot_lidar_data.at(idx).at(i));
      lidar_derivative.push_back(lidar_derivatives.at(idx).at(i));
    }
    std::vector<float> ray_pos, average_depth;
    for (auto [ray, depth]: obstacle_lists.at(idx)) {
      ray_pos.push_back(ray);
      average_depth.push_back(depth);
    }
    std::vector<std::string> labels = {"Lidar Data", "Lidar Derivative", "Obstacles"};
    std::vector<cv::Scalar> colors = {red, blue, green};


    auto width = computeImageDimensions(x_axis, ray_pos);
    auto height = computeImageDimensions(lidar_derivative, lidar_data, average_depth);

    cv::Mat lidar_image = cv::Mat::zeros(height, width, CV_8UC3);
    plotLineGraph(lidar_image, x_axis, lidar_data, red, height);
    plotLineGraph(lidar_image, x_axis, lidar_derivative, blue, height);
    plotGraphPoints(lidar_image, ray_pos, average_depth, green, height);
    drawLegend(lidar_image, labels, colors);

    cv::imshow("Lidar Raw Data", lidar_image);
  }
  cv::waitKey(0);

  //Lidar Map
  for (std::size_t idx = 0; idx < lidar_coordinates.size(); idx++) {
    auto lidar_map = lidar_coordinates.at(idx);
    std::vector<float> x, y;
    for (auto [x_coor,y_coor]: lidar_map) {
      x.push_back(x_coor);
      y.push_back(y_coor);
    }
    auto obstacle_pos = obstacle_coordinates.at(idx);
    std::vector<float> x_obs, y_obs;
    for (auto [x,y]: obstacle_pos) {
      x_obs.push_back(x);
      y_obs.push_back(y);
    }
    auto width = computeImageDimensions(x, x_obs);
    auto height = computeImageDimensions(y, y_obs);

    std::vector<std::string> labels = {"Lidar Points", "Obstacles"};
    std::vector<cv::Scalar> colors = {red, green};

    cv::Mat lidar_map_image = cv::Mat::zeros(height, width, CV_8UC3);
    plotLineGraph(lidar_map_image, x, y, red, height);
    plotGraphPoints(lidar_map_image, x_obs, y_obs, green, height);
    drawLegend(lidar_map_image, labels, colors);
    cv::imshow("Lidar Map", lidar_map_image);
    cv::waitKey(150);
  }

  return 0;
}
