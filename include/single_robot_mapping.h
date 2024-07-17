/**
 * @file single_robot_mapping.h
 * @author Parth Patel (parth.pmech@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef SINGLE_ROBOT_MAPPING_H
#define SINGLE_ROBOT_MAPPING_H

#include <algorithm>
#include <iostream>
#include <process_sensor_data.h>
#include <vector>
#include <Eigen/Dense>

enum class cellState {
    EMPTY = 0,
    UNKNOWN = 1,
    FILLED = 2
};

typedef Eigen::Matrix<cellState, Eigen::Dynamic, Eigen::Dynamic> Grid;

/**
 * @brief
 * TODO: Implement Interface to take in Sensor Data
 * TODO: Implement Functionality to use LIDAR Data with ODOM to estimate Occupancy Cell
 * TODO: Implement Loop Closure Functionality
 * TODO: Implement Interface to enable Multi-robot SLAM
 */
class map_environment {
public:
    map_environment(const float &total_grid_length_millimeters, const float &grid_cell_length_millimeters);

    ~map_environment();

    Grid updateMap(const std::vector<coordinate_t> &lidarScan, const pose_t &current_pose) {
        for (size_t i = 0; i < lidarScan.size(); i++) {
            auto [x_scan, y_scan] = lidarScan.at(i);
            if (x_scan != -1.0F) {
                auto x = current_pose.x + x_scan;
                auto y = current_pose.y + y_scan;
                if ((2.0F * x > this->m_totalGridLength_millimeters) || (
                        2.0F * y > this->m_totalGridLength_millimeters)) {
                    float new_length = std::max(x, y);
                    new_length *= 4.0f;
                    resize_grid(new_length);
                }
                const int x_off = this->m_Offset - static_cast<int>(x / this->m_gridCellSize_millimeters);
                const int y_off = this->m_Offset - static_cast<int>(y / this->m_gridCellSize_millimeters);
                this->m_gridMap(y_off, x_off) = cellState::FILLED; // Use parentheses for Eigen matrix element access
            }
        }
        return this->m_gridMap;
    }


    /* --------------------------------- Getters -------------------------------- */
    [[nodiscard]] const float getGridLength() const noexcept { return this->m_totalGridLength_millimeters; }
    [[nodiscard]] const Grid &getGridMap() const noexcept { return this->m_gridMap; }
    [[nodiscard]] const size_t &getGridCellCount() const noexcept { return this->m_cellCount; }

    bool resize_grid(const float &new_grid_length_millimeters);

    int m_Offset;

private:
    /* ---------------------------- Member Functions ---------------------------- */
    std::size_t calculate_grid_cell_count(const float &grid_length_millimeters);

    /* ---------------------------- Member Variables ---------------------------- */
    float m_totalGridLength_millimeters;
    float m_gridCellSize_millimeters;
    size_t m_cellCount;
    Grid m_gridMap;
};

#endif // SINGLE_ROBOT_MAPPING_H
