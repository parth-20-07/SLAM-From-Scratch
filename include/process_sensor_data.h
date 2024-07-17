/**    plt::named_plot("processed lidar " + std::to_string(idx), processed_lidar_data.at(idx), "r");

 * @file process_sensor_data.h
 * @brief Header file for processing sensor data.
 * @version 0.1
 * @date 2024-06-06
 */

#ifndef PROCESS_SENSOR_DATA_H
#define PROCESS_SENSOR_DATA_H

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

typedef struct {
    float ray_position;
    float average_depth;
} obstacle_location_t;

typedef struct {
    float x;
    float y;
} coordinate_t;

/**
 * @brief Class for processing LIDAR data.
 */
class lidar_data {
public:
    lidar_data(
        std::size_t number_of_lidar_data_points,
        value_range_t scan_distance,
        value_range_t angle_range_Radians,
        float detection_threshold);

    ~lidar_data();

    [[nodiscard]] std::vector<coordinate_t> process_lidar_scan(const std::vector<float> &lidar_scan,
                                                               const pose_t current_pose);

    [[nodiscard]] std::vector<coordinate_t> get_obstacle_coordinates(void);

private:
    const std::size_t m_totalLidarDataPoints; // Total LIDAR data points
    const value_range_t m_scanDistance; // LIDAR scan distance range
    const value_range_t m_angleRange_Radians; // LIDAR angle range
    const float m_detectionThreshold; //Threshold Value to Consider an object as obstacle
    const float m_anglePerRayIncrement_Radians;
    std::vector<coordinate_t> m_obstacle_coordinates;

private:
    coordinate_t m_convert_ray_to_position(pose_t current_pose, float ray_id, float ray_value) const;

    [[nodiscard]] std::vector<float> calculate_derivative(const std::vector<float> &data) const;

    [[nodiscard]] std::vector<obstacle_location_t> find_obstacles(
        const std::vector<float> &data,
        const std::vector<float> &derivative_data) const;

    [[nodiscard]] std::vector<coordinate_t> convert_obstacle_to_coordinate_in_lidar_frame(
        const std::vector<obstacle_location_t> &obstacles,
        const pose_t currentPose) const;

    [[nodiscard]] std::vector<coordinate_t> convert_scan_to_coordinate_in_lidar_frame(const std::vector<float> &data,
                                                                       const pose_t currentPose) const;
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
    [[nodiscard]] pose_t transformFrame(pose_t currentPose) const; // Declaration without extra qualification


private:
    const float m_encoderTicksPerMillimeter;
    const float m_robotWidthBetweenWheels_millimeters;
    encoder_ticks_t m_currentEncoderTickValue;
    pose_t m_robotPose;
    pose_t m_transformationMatrixToSwitchFrame;

private:
    void m_calculate_motion(float dL, float dR);
};

#endif // PROCESS_SENSOR_DATA_H
