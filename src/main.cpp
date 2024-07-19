
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
#include "matplotlibcpp.h"
#include <fmt/core.h>

namespace fs = std::filesystem;
namespace plt = matplotlibcpp;

cv::Scalar red(0, 0, 255);
cv::Scalar green(0, 255, 0);
cv::Scalar blue(255, 0, 0);
cv::Scalar white(255, 255, 255);

std::unique_ptr<lidar_data> lidarObject;
std::unique_ptr<robot_odometry> robotOdomObject;
std::unique_ptr<map_environment> robotMap;
constexpr float grid_size = 10.0f;

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

void plotGraphPoints(cv::Mat &image, const std::vector<pose_t> &pose, const cv::Scalar &color, int height) {
    for (const auto &[x, y, theta]: pose) {
        const int x_off = robotMap->m_center + static_cast<int>(x / grid_size);
        const int y_off = height - (robotMap->m_center + static_cast<int>(y / grid_size)) - 1;
        cv::circle(image, cv::Point(x_off, y_off), 2, color, cv::FILLED);

        // Calculate the end point of the heading line
        const int line_length = 10; // length of the line indicating heading
        const int x_end = x_off + static_cast<int>(line_length * cos(theta));
        const int y_end = y_off - static_cast<int>(line_length * sin(theta));

        // Draw the heading line
        cv::line(image, cv::Point(x_off, y_off), cv::Point(x_end, y_end), color, 1);
    }
}


void plotGraphPoints(cv::Mat &image, const std::vector<coordinate_t> &coordinates, const cv::Scalar &color,
                     int height) {
    for (const auto &[x, y]: coordinates) {
        const int x_off = robotMap->m_center + static_cast<int>(x / grid_size);
        const int y_off = height - (robotMap->m_center + static_cast<int>(y / grid_size)) - 1;
        cv::circle(image, cv::Point(x_off, y_off), 10, color, cv::FILLED);
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

void plotLineGraph(cv::Mat &image, const std::vector<coordinate_t> &coordinates, const cv::Scalar &color, int height) {
    // Draw lines
    for (size_t i = 1; i < coordinates.size(); ++i) {
        auto [x1, y1] = coordinates.at(i - 1);
        auto [x2, y2] = coordinates.at(i);
        const int xCoord1 = height - static_cast<int>(x1);
        const int yCoord1 = height - static_cast<int>(y1);
        const int xCoord2 = height - static_cast<int>(x2);
        const int yCoord2 = height - static_cast<int>(y2);
        cv::line(image, cv::Point(xCoord1, yCoord1), cv::Point(xCoord2, yCoord2), color, 5);
    }
}


void plotLinesObstacles(cv::Mat &image, const std::vector<coordinate_t> &coordinates, const cv::Scalar &color,
                        int height) {
    // Draw lines
    for (size_t i = 1; i < coordinates.size(); ++i) {
        auto [x1, y1] = coordinates.at(i - 1);
        auto [x2, y2] = coordinates.at(i);
        const int xCoord1 = robotMap->m_center + static_cast<int>(x1 / grid_size);
        const int yCoord1 = height - (robotMap->m_center + static_cast<int>(y1 / grid_size)) - 1;
        const int xCoord2 = robotMap->m_center + static_cast<int>(x2 / grid_size);
        const int yCoord2 = height - (robotMap->m_center + static_cast<int>(y2 / grid_size)) - 1;
        cv::line(image, cv::Point(xCoord1, yCoord1), cv::Point(xCoord2, yCoord2), color, 2);
    }
}

void gridToImage(cv::Mat &image, const Grid &grid) {
    int rows = grid.rows();
    int cols = grid.cols();
    image = cv::Mat(rows, cols, CV_8UC3);

    // Define colors for each cell state
    static const cv::Vec3b emptyColor(255, 255, 255); // White for EMPTY
    static const cv::Vec3b unknownColor(127, 127, 127); // Grey for UNKNOWN
    static const cv::Vec3b filledColor(0, 0, 0); // Black for FILLED

    // Populate the image
    for (int y = 0; y < rows; y++) {
        const int y_val = rows - y - 1;
        for (int x = 0; x < cols; x++) {
            // Ensure y and x are within bounds
            if (y < rows && x < cols) {
                switch (grid(y, x)) {
                    case cellState::EMPTY:
                        image.at<cv::Vec3b>(y_val, x) = emptyColor;
                        break;
                    case cellState::UNKNOWN:
                        image.at<cv::Vec3b>(y_val, x) = unknownColor;
                        break;
                    case cellState::FILLED:
                        image.at<cv::Vec3b>(y_val, x) = filledColor;
                        break;
                }
            } else {
                std::cerr << "Index out of bounds: (" << y << ", " << x << ")" << std::endl;
            }
        }
    }
}


void drawLegend(cv::Mat &image, const std::vector<std::string> &labels, const std::vector<cv::Scalar> &colors,
                int x = 50, int y = 50) {
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;

    for (size_t i = 0; i < labels.size(); ++i) {
        constexpr int thickness = 1;
        constexpr double fontScale = 0.5;
        cv::putText(image, labels[i], cv::Point(x + 20, y + i * 20), fontFace, fontScale, white, thickness);
        cv::rectangle(image, cv::Point(x, y + i * 20 - 10), cv::Point(x + 10, y + i * 20), colors[i], cv::FILLED);
    }
}

std::vector<pose_t> all_pose_robot_global_frame;
std::vector<std::vector<coordinate_t> > obstacles_in_lidar_frame;
cv::Mat slam_map_image;

void step(int ts, const encoder_ticks_t encoderTick, const std::vector<float> &lidar_data, int idx, bool plot) {
    const pose_t current_pose_global_frame = robotOdomObject->m_update_pose(encoderTick);

    const auto [lidar_map_robot_frame, obstacles_robot_frame, obstacle_lidar_frame] = lidarObject->process_lidar_scan_to_robot_frame(
        lidar_data, current_pose_global_frame);
    auto currentMap_robot_origin_frame = robotMap->updateMap_robot_origin_frame(
        obstacles_robot_frame, current_pose_global_frame);

    all_pose_robot_global_frame.push_back(current_pose_global_frame);
    obstacles_in_lidar_frame.push_back(obstacle_lidar_frame);

    const auto global_frame_obstacles = lidarObject->get_coordinates_in_robot_origin_frame(
        obstacles_robot_frame, current_pose_global_frame);
    const auto global_frame_lidar = lidarObject->get_coordinates_in_robot_origin_frame(
        lidar_map_robot_frame, current_pose_global_frame);


    // std::cout << "Frame: " << std::to_string(idx) << "\n";
    gridToImage(slam_map_image, currentMap_robot_origin_frame);
    plotGraphPoints(slam_map_image, all_pose_robot_global_frame, red, slam_map_image.cols);
    plotLinesObstacles(slam_map_image, global_frame_obstacles, green, slam_map_image.cols);
    plotGraphPoints(slam_map_image, global_frame_obstacles, blue, slam_map_image.cols);
    cv::imshow("Lidar Map", slam_map_image);
    cv::waitKey(10);
    if (plot) {
        // Plot with matplotlib
        plt::figure_size(800, 800);
        std::vector<float> x, y;
        for (auto scan: global_frame_lidar) {
            x.push_back(scan.x);
            y.push_back(scan.y);
        }
        plt::named_plot("lidar ", x, y, "b");
        std::vector<float> x_obs, y_obs;
        for (auto obs: global_frame_obstacles) {
            x_obs.push_back(obs.x);
            y_obs.push_back(obs.y);
        }
        plt::named_plot("Obstacles ", x_obs, y_obs, "y-o");
        plt::named_plot("Current Pose", std::vector<float>{current_pose_global_frame.x},
                        std::vector<float>{current_pose_global_frame.y}, "r-o");
        plt::legend();
        plt::xlabel("position");
        plt::ylabel("Sensor Reading");
        plt::title("Lidar Data " + std::to_string(idx));
        plt::grid(true);
        plt::show();
    }
}

int main(int argc, char **argv) {
    // Get the Data Path
    std::string path = __FILE__;
    auto pos = path.find_last_of('/');
    if (pos != std::string::npos) {
        path = path.substr(0, pos);
    }
    pos = path.find_last_of('/');
    if (pos != std::string::npos) {
        path = path.substr(0, pos);
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
        });

    //Lidar Scanning
    lidarObject = std::make_unique<lidar_data>(
        robot_lidar_data.at(0).size(),
        value_range_t{20.0F, 2200.0F},
        value_range_t{-2.0946678100889633F, 1.9489055467778735F},
        100.0F,
        pose_t{
            .x = 0.0F,
            .y = 30.0F,
            .theta = 0.0F
        },
        90.0F,
        -0.06981317007977318F);

    // Environment Map
    robotMap = std::make_unique<map_environment>(100.0, grid_size);

    /////////////////////////// Run SLAM ///////////////////////////////////////
    for (std::size_t i = 0; i < motor_ticks.size(); i++) {
        //Encoder Data
        auto motor_tick = motor_ticks.at(i);
        int ts = motor_tick.at(0);
        int left = motor_tick.at(1);
        int right = motor_tick.at(2);
        encoder_ticks_t encoderVal{
            .left = static_cast<float>(left),
            .right = static_cast<float>(right)
        };

        auto lidar_scan = robot_lidar_data.at(i); //LIDAR Data

        step(ts, encoderVal, lidar_scan, i, false);
    }

    /////////////////////////// Save Data ///////////////////////////////////////
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
    //Motor Ticks
    {
        std::string path = results_dir;
        path.append("/poses_from_ticks.txt");
        std::ofstream file(path, std::ios::out);
        if (!file.is_open()) {
            std::cerr << "Failed to open file for writing: " << path << std::endl;
        } else {
            for (auto [x,y,theta]: all_pose_robot_global_frame) {
                file << fmt::format("F\t{:5f}\t{:5f}\t{:5f}\n", x, y, theta);
            }
            file.close();
            std::cout << "Data written to file: " << path << std::endl;
        }
    }

    //Obstacle Positions
    {
        std::string path = results_dir;
        path.append("/cylinders.txt");
        std::ofstream file(path, std::ios::out);
        if (!file.is_open()) {
            std::cerr << "Failed to open file for writing: " << path << std::endl;
        } else {
            for (auto obstacles: obstacles_in_lidar_frame) {
                file << "D C\t";
                for (auto [x,y]: obstacles) {
                    file << fmt::format("{:5f}\t{:5f}\t", x, y);
                }
                file << "\n";
            }
            file.close();
            std::cout << "Data written to file: " << path << std::endl;
        }
    }
    return 0;
}
