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

#include <iostream>
#include <vector>
#include <stdint.h>
enum class cellState
{
    EMPTY = 0,
    UNKNOWN = 1,
    FILLED = 2
};

typedef std::vector<std::vector<cellState>> Grid;

/**
 * @brief
 * TODO: Implement Interface to take in Sensor Data
 * TODO: Implement Functionality to use LIDAR Data with ODOM to estimate Occupancy Cell
 * TODO: Implement Loop Closure Functionality
 * TODO: Implement Interface to enable Multi-robot SLAM
 */
class map_environment
{
public:
    map_environment(const float &total_grid_length_millimeters, const float &grid_cell_length_millimeters);
    ~map_environment();

    /* --------------------------------- Getters -------------------------------- */
    [[nodiscard]] const float getGridLength() const noexcept { return this->m_totalGridLength_millimeters; }
    [[nodiscard]] const Grid &getGridMap() const noexcept { return this->m_gridMap; }
    [[nodiscard]] const size_t &getGridCellCount() const noexcept { return this->m_cellCount; }

    bool resize_grid(const float &new_grid_length_millimeters);

private:
    /* ---------------------------- Member Functions ---------------------------- */
    std::size_t calculate_grid_cell_count(const float &grid_length_millimeters);

    /* ---------------------------- Member Variables ---------------------------- */
    float m_totalGridLength_millimeters;
    float m_gridSize_millimeters;
    size_t m_cellCount;
    Grid m_gridMap;
};

#endif // SINGLE_ROBOT_MAPPING_H
