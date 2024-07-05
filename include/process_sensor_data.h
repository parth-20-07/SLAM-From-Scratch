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
#include <cstdint>
#include<memory>
#include <vector>

typedef struct {
    float min;
    float max;
} value_range_t;

typedef struct {
    float x;
    float y;
    float theta;
} pose_t;

typedef struct {
    float left;
    float right;
} encoder_ticks_t;

/**
 * @brief
 * TODO: Implement Interface to feed data for SLAM
 *
 */
class lidar_data {
private:
    const std::size_t m_totalLidarDataPoints; // Angle Increment is assumed Uniform
    const value_range_t m_scanDistance // Lidar Range
    {
        .min{0.0F}, .max{0.0F}
    };
    const value_range_t m_angleRange // Start and End Angle for LIDAR
    {
        .min{0.0F}, .max{360.0F}
    };

public:
    lidar_data(
        std::size_t number_of_lidar_data_points,
        value_range_t scan_distance,
        value_range_t angle_range);

    ~lidar_data();

    std::vector<float> m_process_data(const std::vector<float> &data) const;
};

/**
 * @brief
 * TODO: Implement Functionality for Robot Odometry
 */
class robot_odometry {
public:
    robot_odometry(
        float encoder_ticks_per_mm,
        float distance_between_wheels_in_mm,
        pose_t starting_pose = pose_t{0.0F, 0.0F, 0.0F},
        encoder_ticks_t starting_encoder_ticks = encoder_ticks_t{0.0F, 0.0F});

    ~robot_odometry();

    [[nodiscard]] pose_t get_current_pose() const { return this->m_robotPose; }

    void m_update_pose(encoder_ticks_t new_encoder_ticks); // Uses Encoder Ticks as Input
    void m_update_pose(float x, float y, float theta); // Just reformats the pose in required format

private:
    const float m_encoderTicksPerMillimeter;
    const float m_robotWidthBetweenWheels_millimeters;
    // const float m_encoderNoise{0.05F};
    // const float m_acceptableErrorInEncoder{0.1F};
    encoder_ticks_t m_currentEncoderTickValue;
    pose_t m_robotPose;

private:
    void m_calculate_motion(float dL, float dR);

    /* ---------------------------- Member Variables ---------------------------- */
};

#endif // PROCESS_SENSOR_DATA_H
