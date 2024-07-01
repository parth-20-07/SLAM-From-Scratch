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
map_environment ::map_environment(const float &total_grid_length_millimeters, const float &grid_cell_length_millimeters)
    : m_totalGridLength_millimeters(total_grid_length_millimeters),
      m_gridSize_millimeters(grid_cell_length_millimeters)
{
    this->m_cellCount = calculate_grid_cell_count(this->m_totalGridLength_millimeters);

    this->m_gridMap.reserve(this->m_cellCount);
    for (size_t i = 0; i < this->m_cellCount; i++)
    {
        this->m_gridMap.emplace_back(this->m_cellCount, cellState::UNKNOWN);
    }
}

map_environment ::~map_environment()
{
}

bool map_environment::resize_grid(const float &new_grid_length_millimeters)
{
    if (new_grid_length_millimeters > this->m_totalGridLength_millimeters)
    {
        this->m_totalGridLength_millimeters = new_grid_length_millimeters;
        size_t newCellCount{calculate_grid_cell_count(new_grid_length_millimeters)};

        Grid newGrid;
        newGrid.reserve(newCellCount);
        for (std::size_t i = 0; i < newCellCount; i++)
        {
            newGrid.emplace_back(newCellCount, cellState::UNKNOWN);
        }

        int offset = (newCellCount - this->m_cellCount) / 2.0;

        for (std::size_t y = 0; y < this->m_cellCount; ++y)
            for (std::size_t x = 0; x < this->m_cellCount; ++x)
                newGrid.at(y + offset).at(x + offset) = this->m_gridMap.at(y).at(x);

        std::swap(this->m_gridMap, newGrid);
        this->m_cellCount = newCellCount;
        std::cout << "Resize Successful!!" << "\nCurrent size is: " << static_cast<int>(this->m_totalGridLength_millimeters) << "\n.New Requested Grid Size: " << static_cast<int>(new_grid_length_millimeters) << std::endl;
        return true;
    }
    else
    {
        std::cout << "Cannot Resize the Grid." << "\nCurrent size is: " << static_cast<int>(this->m_totalGridLength_millimeters) << "\n.New Requested Grid Size: " << static_cast<int>(new_grid_length_millimeters) << std::endl;
        return false;
    }
}

std::size_t map_environment::calculate_grid_cell_count(const float &grid_length_millimeters)
{
    return static_cast<std::size_t>(grid_length_millimeters / this->m_gridSize_millimeters);
}
