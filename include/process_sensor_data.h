/**
 * @file process_sensor_data.h
 * @author Parth Patel (parth.pmech@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef PROCESS_SENSOR_DATA_H
#define PROCESS_SENSOR_DATA_H
#include <iostream>
#include <stdint.h>
#include <vector>
#include <limits>
#include <cmath>

struct value_range
{
    float min;
    float max;
};

struct pose
{
    float x;
    float y;
    float theta;
};

struct encoder_ticks
{
    float left;
    float right;
};

struct encoder_rot
{
    double left_wheel_val;
    double right_wheel_val;
};

using pose_t = struct pose;
using value_range_t = struct value_range;
using encoder_ticks_t = struct encoder_ticks;
using encoder_rot_t = struct encoder_rot;

/**
 * @brief
 * TODO: Implement Interface to feed data for SLAM
 *
 */
class lidar_data
{
private:
    /* ---------------------------- Member Functions ---------------------------- */

    /* ---------------------------- Member Variables ---------------------------- */
    const std::size_t m_totalLidarDataPoints; // Angle Increment is assumed Uniform
    const value_range_t m_scanDistance        // Lidar Range
        {
            .min{0.0F}, .max{0.0F}};
    const value_range_t m_angleRange // Start and End Angle for LIDAR
        {
            .min{0.0F}, .max{360.0F}};

public:
    /* ---------------------------- Member Functions ---------------------------- */
    lidar_data(
        std::size_t number_of_lidar_data_points,
        value_range_t scan_distance,
        value_range_t angle_range);
    ~lidar_data();

    std::vector<float> m_process_data(std::vector<float> &data);
    /* ---------------------------- Member Variables ---------------------------- */
};

/**
 * @brief
 * TODO: Implement Functionality for Robot Odometry
 */
class robot_odometry
{
private:
    /* ---------------------------- Member Functions ---------------------------- */
    const std::uint32_t m_encoderCountPerRevolution;
    const float m_wheelRadius_millimeters;
    const float m_robotWidthBetweenWheels_millimeters;
    const float m_encoderNoise{0.05F};
    const float m_acceptableErrorInEncoder{0.1F};
    encoder_rot_t m_currentEncoderValue{.left_wheel_val{0.0F}, .right_wheel_val{0.0F}};

    pose_t m_robotPose{.x = 0.0F, .y = 0.0F, .theta = 0.0F};

    pose_t m_calculate_motion(float dL, float dR);

    /* ---------------------------- Member Variables ---------------------------- */
public:
    /* ---------------------------- Member Functions ---------------------------- */
    robot_odometry(
        const std::uint32_t encoder_ticks_per_revolution,
        const float wheel_radius_in_mm,
        const float distance_between_wheels_in_mm);
    ~robot_odometry();

    const pose_t get_current_pose(void) const { return this->m_robotPose; }

    void m_update_pose(const encoder_ticks_t new_encoder_ticks);         // Uses Encoder Ticks as Input
    void m_update_pose(const encoder_rot_t new_absolute_encoder_value);  // Uses the Encoder Revolution as Input
    void m_update_pose(const float x, const float y, const float theta); // Just reformats the pose in required format

    /* ---------------------------- Member Variables ---------------------------- */
};

#endif // PROCESS_SENSOR_DATA_H
