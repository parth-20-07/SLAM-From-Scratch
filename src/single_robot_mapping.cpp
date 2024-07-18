/**
 *
 * @file single_robot_mapping.cpp
 * @author Parth Patel (parth.pmech@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "single_robot_mapping.h"

map_environment::map_environment(const float &total_grid_length_millimeters, const float &grid_cell_length_millimeters)
    : m_totalGridLength_millimeters(total_grid_length_millimeters),
      m_gridCellSize_millimeters(grid_cell_length_millimeters),
      m_cellCount(static_cast<int>(total_grid_length_millimeters / grid_cell_length_millimeters)),
      m_Offset(static_cast<int>(m_cellCount / 2.0F)),
      m_gridMap(m_cellCount, m_cellCount) {
    m_gridMap.setConstant(cellState::UNKNOWN);
}

map_environment::~map_environment() {
}

bool map_environment::resize_grid(const float &new_grid_length_millimeters) {
    if (new_grid_length_millimeters > this->m_totalGridLength_millimeters) {
        size_t newCellCount{calculate_grid_cell_count(new_grid_length_millimeters)};

        // Create a new grid with the new size and initialize it with UNKNOWN
        Grid newGrid(newCellCount, newCellCount);
        newGrid.setConstant(cellState::UNKNOWN);

        int offset = (newCellCount - this->m_cellCount) / 2;

        // Copy the old grid into the new grid with the new offset
        for (std::size_t y = 0; y < this->m_cellCount; ++y) {
            for (std::size_t x = 0; x < this->m_cellCount; ++x) {
                newGrid(y + offset, x + offset) = this->m_gridMap(y, x);
            }
        }

        // Update the grid map with the new grid
        std::swap(this->m_gridMap, newGrid);
        this->m_cellCount = newCellCount;
        std::cout << "Resize Successful!!" << "\nCurrent size is: " << static_cast<int>(this->
                    m_totalGridLength_millimeters)
                << "\nNew Requested Grid Size: " << static_cast<int>(new_grid_length_millimeters) << std::endl;
        this->m_totalGridLength_millimeters = new_grid_length_millimeters;
        this->m_Offset = offset;
        return true;
    } else {
        std::cout << "Cannot Resize the Grid." << "\nCurrent size is: " << static_cast<int>(this->
                    m_totalGridLength_millimeters)
                << "\n.New Requested Grid Size: " << static_cast<int>(new_grid_length_millimeters) << std::endl;
        return false;
    }
}


std::size_t map_environment::calculate_grid_cell_count(const float &grid_length_millimeters) {
    return static_cast<std::size_t>(grid_length_millimeters / this->m_gridCellSize_millimeters);
}

Grid map_environment::updateMap(const std::vector<coordinate_t> &lidarScan, const pose_t &current_pose) {
    for (size_t i = 0; i < lidarScan.size(); i++) {
        auto [x_scan, y_scan] = lidarScan.at(i);
        if (x_scan != -1.0F) {
            // Apply rotation if needed
            float theta = current_pose.theta;
            float x = current_pose.x + (x_scan * cos(theta) - y_scan * sin(theta));
            float y = current_pose.y + (x_scan * sin(theta) + y_scan * cos(theta));

            // Check grid boundaries and resize if necessary
            if ((2.0F * x > this->m_totalGridLength_millimeters) || (2.0F * y > this->m_totalGridLength_millimeters)) {
                float new_length = std::max(x, y);
                new_length *= 4.0f;
                resize_grid(new_length);
            }

            // Convert to grid indices
            int x_off = m_Offset + static_cast<int>(x / this->m_gridCellSize_millimeters);
            int y_off = m_Offset + static_cast<int>(y / this->m_gridCellSize_millimeters);
            m_gridMap(y_off, x_off) = cellState::FILLED;
        }
    }
    return this->m_gridMap;
}
