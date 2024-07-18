/**
 * @file process_sensor_data.cpp
 * @brief Implementation file for processing sensor data.
 * @version 0.1
 * @date 2024-06-06
 */

#include "process_sensor_data.h"
#include <cmath>

// Constructor for lidar_data
lidar_data::lidar_data(
    std::size_t number_of_lidar_data_points,
    value_range_t scan_distance,
    value_range_t angle_range_Radians,
    float detection_threshold)
    : m_totalLidarDataPoints(number_of_lidar_data_points),
      m_scanDistance(scan_distance),
      m_angleRange_Radians{angle_range_Radians},
      m_detectionThreshold(detection_threshold),
      m_anglePerRayIncrement_Radians((m_angleRange_Radians.max - m_angleRange_Radians.min) / m_totalLidarDataPoints) {
}

// Destructor for lidar_data
lidar_data::~lidar_data() = default;

std::vector<float> lidar_data::calculate_derivative(const std::vector<float> &data) const {
    std::vector<float> derivative;
    derivative.resize(data.size());
    derivative.at(0) = 0.0F;
    for (std::size_t idx = 1; idx < data.size() - 2; idx++) {
        float left = data.at(idx - 1);
        float right = data.at(idx + 1);
        if (right > m_scanDistance.min && left > m_scanDistance.min) {
            float der = (right - left) / 2.0F;
            if (der <= -m_detectionThreshold)
                der = -m_detectionThreshold;
            else if (der >= m_detectionThreshold)
                der = m_detectionThreshold;
            else
                der = 0.0F;
            derivative.at(idx) = der;
        } else {
            derivative.at(idx) = 0.0F;
        }
    }
    derivative.at(data.size() - 1) = 0.0F;
    return derivative;
}

std::vector<obstacle_location_t> lidar_data::find_obstacles(
    const std::vector<float> &data,
    const std::vector<float> &derivative_data) const {
    std::vector<obstacle_location_t> obstacles;

    for (std::size_t idx = 0; idx < data.size(); idx++) {
        if (derivative_data.at(idx) < 0.0F) {
            //Low Trigger Detected for Obstacle
            obstacle_location_t obstacle;
            int num_of_rays = 0;
            int sum_of_rays = 0;
            float depth_sum = 0.0F;
            while (idx < data.size() && derivative_data.at(idx) <= 0) {
                if (data.at(idx) > m_scanDistance.min) {
                    num_of_rays++;
                    sum_of_rays += static_cast<int>(idx);
                    depth_sum += data.at(idx);
                }
                idx++;
            }

            if (idx == data.size())
                break;
            if (derivative_data.at(idx) > 0) {
                //Obstacle Reading Complete
                obstacle.average_depth = depth_sum / static_cast<float>(num_of_rays);
                obstacle.ray_position = static_cast<float>(sum_of_rays) / static_cast<float>(num_of_rays);
                obstacles.push_back(obstacle);
            } else {
                //Detected Another Obstacle in front of current one
                // idx--;
            }
        }
    }
    return obstacles;
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::convert_obstacle_to_coordinate_in_lidar_frame(
    const std::vector<obstacle_location_t> &obstacles,
    const pose_t currentPose) const {
    std::vector<coordinate_t> obstacle_positions;
    for (std::size_t idx = 0; idx < obstacles.size(); idx++) {
        auto obstacle = obstacles.at(idx);
        float ray_id = obstacle.ray_position;
        float ray_value = obstacle.average_depth;
        obstacle_positions.emplace_back(this->m_convert_ray_to_position(
            currentPose,
            ray_id,
            ray_value));
    }

    return obstacle_positions;
}


[[nodiscard]] std::vector<coordinate_t> lidar_data::convert_scan_to_coordinate_in_lidar_frame(
    const std::vector<float> &data,
    const pose_t currentPose) const {
    std::vector<coordinate_t> positions;
    for (std::size_t idx = 0; idx < data.size(); idx++) {
        float ray_id = idx;
        float ray_value = data.at(idx);
        if (ray_value < this->m_scanDistance.max) {
            positions.emplace_back(this->m_convert_ray_to_position(
                currentPose,
                ray_id,
                ray_value));
        } else {
            constexpr coordinate_t unused{-1.0F, -1.0F};
            positions.emplace_back(unused);
        }
    }

    return positions;
}

[[nodiscard]] std::pair<std::vector<coordinate_t>, std::vector<coordinate_t> > lidar_data::process_lidar_scan(
    const std::vector<float> &lidar_scan,
    const pose_t current_pose) {
    const auto derivative = this->calculate_derivative(lidar_scan);
    const auto obstacles = this->find_obstacles(lidar_scan, derivative);
    const auto lidar_map = this->convert_scan_to_coordinate_in_lidar_frame(lidar_scan, current_pose);
    m_obstacle_coordinates = this->convert_obstacle_to_coordinate_in_lidar_frame(obstacles, current_pose);

    return std::pair<std::vector<coordinate_t>, std::vector<coordinate_t> >(lidar_map, m_obstacle_coordinates);
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::get_obstacle_coordinates(void) {
    return m_obstacle_coordinates;
}


coordinate_t lidar_data::m_convert_ray_to_position(pose_t current_pose, float ray_id, float ray_value) const {
    const float alpha = ray_id * m_anglePerRayIncrement_Radians; // Angle of the current ray
    static const float half_FoV = (static_cast<float>(m_totalLidarDataPoints) / 2.0F) * m_anglePerRayIncrement_Radians;
    const float gamma = half_FoV - alpha; // Adjusting alpha to the lidar frame

    const float dX = ray_value * std::cos(gamma);
    const float dY = ray_value * std::sin(gamma);

    return coordinate_t{dX, dY};
}


// Constructor for robot_odometry
robot_odometry::robot_odometry(
    const float encoder_ticks_per_mm,
    const float distance_between_wheels_in_mm,
    const pose_t starting_pose,
    const encoder_ticks_t starting_encoder_ticks,
    const pose_t transform_to_another_frame)
    : m_encoderTicksPerMillimeter(encoder_ticks_per_mm),
      m_robotWidthBetweenWheels_millimeters(distance_between_wheels_in_mm),
      m_currentEncoderTickValue(starting_encoder_ticks),
      m_robotPose(starting_pose),
      m_transformationMatrixToSwitchFrame(transform_to_another_frame) {
}

// Destructor for robot_odometry
robot_odometry::~robot_odometry() = default;

// Get current pose
pose_t robot_odometry::get_current_pose() const {
    pose_t transformed_pose = transformFrame(this->m_robotPose);
    return transformed_pose;
}

// Update pose using x, y, and theta
void robot_odometry::m_update_pose(const float x, const float y, const float theta) {
    this->m_robotPose.x = x;
    this->m_robotPose.y = y;
    this->m_robotPose.theta = theta;
}

// Update pose using encoder ticks
pose_t robot_odometry::m_update_pose(const encoder_ticks_t new_encoder_ticks) {
    const float dL = (new_encoder_ticks.left - this->m_currentEncoderTickValue.left) * this->
                     m_encoderTicksPerMillimeter;
    const float dR = (new_encoder_ticks.right - this->m_currentEncoderTickValue.right) * this->
                     m_encoderTicksPerMillimeter;
    this->m_calculate_motion(dL, dR);
    this->m_currentEncoderTickValue = new_encoder_ticks;
    return this->m_robotPose;
}

// Calculate motion based on wheel distances
void robot_odometry::m_calculate_motion(const float dL, const float dR) {
    float newTheta, newX, newY;

    if (dL == dR) {
        // Motion is translational
        newX = this->m_robotPose.x + (dL * std::cos(this->m_robotPose.theta));
        newY = this->m_robotPose.y + (dL * std::sin(this->m_robotPose.theta));
        newTheta = this->m_robotPose.theta;
    } else {
        // Motion is rotational
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

// Transform frame
pose_t robot_odometry::transformFrame(pose_t currentPose) const {
    float cos_theta = std::cos(m_transformationMatrixToSwitchFrame.theta);
    float sin_theta = std::sin(m_transformationMatrixToSwitchFrame.theta);
    float transformedX = cos_theta * currentPose.x - sin_theta * currentPose.y + m_transformationMatrixToSwitchFrame.x;
    float transformedY = sin_theta * currentPose.x + cos_theta * currentPose.y + m_transformationMatrixToSwitchFrame.y;
    float transformedTheta = currentPose.theta + m_transformationMatrixToSwitchFrame.theta;

    transformedTheta = std::fmod(transformedTheta, 2.0F * M_PI);
    if (transformedTheta < 0) {
        transformedTheta += 2.0F * M_PI;
    }

    return pose_t{transformedX, transformedY, transformedTheta};
}
