
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

void gridToImage(cv::Mat &image, const Grid &grid) {
    int rows = grid.rows();
    int cols = grid.cols();
    image = cv::Mat(rows, cols, CV_8UC3);

    // Define colors for each cell state
    static const cv::Vec3b emptyColor(255, 255, 255); // White for EMPTY
    static const cv::Vec3b unknownColor(127, 127, 127); // Grey for UNKNOWN
    static const cv::Vec3b filledColor(0, 0, 0); // Black for FILLED

    // Populate the image
    for (int y = 0; y < rows; ++y) {
        float y_val = rows - y - 1;
        for (int x = 0; x < cols; ++x) {
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

    for (size_t i = 0; i < labels.size(); ++i) {
        constexpr int thickness = 1;
        constexpr double fontScale = 0.5;
        cv::putText(image, labels[i], cv::Point(x + 20, y + i * 20), fontFace, fontScale, white, thickness);
        cv::rectangle(image, cv::Point(x, y + i * 20 - 10), cv::Point(x + 10, y + i * 20), colors[i], cv::FILLED);
    }
}

std::vector<std::vector<coordinate_t> > lidar_coordinates;
std::vector<pose_t> all_pose;
std::vector<Grid> all_maps;

void step(int ts, const encoder_ticks_t encoderTick, const std::vector<float> &lidar_data) {
    robotOdomObject->m_update_pose(encoderTick);
    const pose_t current_pose = robotOdomObject->get_current_pose();

    const auto lidar_map = lidarObject->process_lidar_scan(lidar_data, current_pose);
    auto currentMap = robotMap->updateMap(lidar_map, current_pose);
    all_pose.push_back(current_pose);
    lidar_coordinates.push_back(lidar_map);
    all_maps.push_back(currentMap);
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
        value_range_t{20.0F,300.0F},
        value_range_t{-2.09466781009F, 1.95504146993F},
        100.0F
    );

    // Environment Map
    robotMap = std::make_unique<map_environment>(100.0, 5.0);

    /////////////////////////// Run SLAM ///////////////////////////////////////
    for (std::size_t i = 0; i < motor_ticks.size(); i++) {
        //Encoder Data
        auto encoderReading = motor_ticks.at(i);

        int timeStep = encoderReading.at(0);
        encoder_ticks_t encoderVal{
            .left = static_cast<float>(encoderReading.at(1)),
            .right = static_cast<float>(encoderReading.at(2))
        };

        //LIDAR Data
        auto lidar_scan = robot_lidar_data.at(i);

        step(timeStep, encoderVal, lidar_scan);
    }

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


    //SLAM Map
    std::vector<float> x_data, y_data, theta_data;
    for (std::size_t idx = 0; idx < all_maps.size(); idx++) {
        auto map = all_maps.at(idx); {
            auto [x,y,_] = all_pose.at(idx);
            const int x_off = robotMap->m_Offset - static_cast<int>(x / 5.0F);
            const int y_off = robotMap->m_Offset - static_cast<int>(y / 5.0F);
            x_data.push_back(x_off);
            y_data.push_back(y_off);
        }

        std::cout << "Processing map " << idx << " / " << all_maps.size() << std::endl;

        cv::Mat image;
        gridToImage(image, map);
        plotGraphPoints(image, x_data, y_data, red, image.cols);
        cv::imshow("Lidar Map", image);
        cv::waitKey(1);
    }

    return 0;
}
