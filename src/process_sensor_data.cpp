/**
 * @file process_sensor_data.cpp
 * @author Parth Patel (parth.pmech@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "process_sensor_data.h"

#include <cmath>
#include <limits>


lidar_data::lidar_data(
    std::size_t number_of_lidar_data_points,
    value_range_t scan_distance,
    value_range_t angle_range)
    : m_totalLidarDataPoints(number_of_lidar_data_points),
      m_scanDistance(scan_distance),
      m_angleRange{angle_range} {
}

lidar_data::~lidar_data() = default;

std::vector<float> lidar_data::m_process_data(const std::vector<float> &data) const {
    std::vector<float> processed_data;
    processed_data.reserve(this->m_totalLidarDataPoints); // Reserve space to avoid reallocations

    for (size_t i = 0; i < this->m_totalLidarDataPoints; i++) {
        float val = data.at(i);
        if (val >= this->m_scanDistance.max) {
            val = std::numeric_limits<float>::max();
        } else if (val <= this->m_scanDistance.min) {
            val = std::numeric_limits<float>::min();
        }
        processed_data.emplace_back(val);
    }

    return std::move(processed_data);
}

robot_odometry::robot_odometry(
    const float encoder_ticks_per_mm,
    const float distance_between_wheels_in_mm,
    const pose_t starting_pose,
    const encoder_ticks_t starting_encoder_ticks)
    : m_encoderTicksPerMillimeter(encoder_ticks_per_mm),
      m_robotWidthBetweenWheels_millimeters(distance_between_wheels_in_mm),
      m_currentEncoderTickValue(starting_encoder_ticks),
      m_robotPose(starting_pose) {
}

robot_odometry::~robot_odometry() = default;

void robot_odometry::m_update_pose(const float x, const float y, const float theta) {
    this->m_robotPose.x = x;
    this->m_robotPose.y = y;
    this->m_robotPose.theta = theta;
}

void robot_odometry::m_update_pose(const encoder_ticks_t new_encoder_ticks) {
    const float dL = (new_encoder_ticks.left - this->m_currentEncoderTickValue.left) * this->
                     m_encoderTicksPerMillimeter;
    const float dR = (new_encoder_ticks.right - this->m_currentEncoderTickValue.right) * this->
                     m_encoderTicksPerMillimeter;
    this->m_calculate_motion(dL, dR);
    this->m_currentEncoderTickValue = new_encoder_ticks;
}


void robot_odometry::m_calculate_motion(const float dL, const float dR) {
    float newTheta, newX, newY;

    if (dL == dR) // Motion is translational
    {
        newX = this->m_robotPose.x + (dL * std::cos(this->m_robotPose.theta));
        newY = this->m_robotPose.y + (dL * std::sin(this->m_robotPose.theta));
        newTheta = this->m_robotPose.theta;
    } else // Motion is Rotational
    {
        const float alpha = (dR - dL) / this->m_robotWidthBetweenWheels_millimeters;
        const float radiusOfRotation = dL / alpha;
        const float RDistance = radiusOfRotation + (this->m_robotWidthBetweenWheels_millimeters / 2.0F);

        const float cX = this->m_robotPose.x - (RDistance * std::sin(this->m_robotPose.theta));
        const float cY = this->m_robotPose.y + (RDistance * std::cos(this->m_robotPose.theta));

        newTheta = std::fmod((this->m_robotPose.theta + alpha), (2.0F * M_PI));
        newX = cX + (RDistance * std::sin(newTheta));
        newY = cY - (RDistance * std::cos(newTheta));
    }

    this->m_robotPose = pose_t{newX, newY, newTheta};
}
