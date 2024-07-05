/**
 * @file process_sensor_data.h
 * @brief Header file for processing sensor data.
 * @version 0.1
 * @date 2024-06-06
 */

#ifndef PROCESS_SENSOR_DATA_H
#define PROCESS_SENSOR_DATA_H

#include <cstdint>
#include <memory>
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
 * @brief Class for processing LIDAR data.
 */
class lidar_data {
private:
    const std::size_t m_totalLidarDataPoints; // Total LIDAR data points
    const value_range_t m_scanDistance // LIDAR scan distance range
    {
        .min{0.0F}, .max{0.0F}
    };
    const value_range_t m_angleRange // LIDAR angle range
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
 * @brief Class for robot odometry.
 */
class robot_odometry {
public:
    robot_odometry(
        float encoder_ticks_per_mm,
        float distance_between_wheels_in_mm,
        pose_t starting_pose,
        encoder_ticks_t starting_encoder_ticks,
        pose_t transform_to_another_frame);

    ~robot_odometry();

    [[nodiscard]] pose_t get_current_pose() const;

    void m_update_pose(encoder_ticks_t new_encoder_ticks); // Uses encoder ticks as input
    void m_update_pose(float x, float y, float theta); // Reformats the pose in required format

private:
    const float m_encoderTicksPerMillimeter;
    const float m_robotWidthBetweenWheels_millimeters;
    encoder_ticks_t m_currentEncoderTickValue;
    pose_t m_robotPose;
    pose_t m_transformationMatrixToSwitchFrame;

private:
    void m_calculate_motion(float dL, float dR);
    [[nodiscard]] pose_t transformFrame(pose_t currentPose) const; // Declaration without extra qualification
};

#endif // PROCESS_SENSOR_DATA_H
