/**
 * @file process_sensor_data.cpp
 * @brief Implementation file for processing sensor data.
 * @version 0.1
 * @date 2024-06-06
 */

#include "process_sensor_data.h"

#include <mutex>
#include <single_robot_mapping.h>
#include <thread>
#include <tuple>

// Constructor for lidar_data
lidar_data::lidar_data(
    std::size_t number_of_lidar_data_points,
    value_range_t scan_distance,
    value_range_t angle_range_Radians,
    float detection_threshold,
    const pose_t transformation_to_robot_frame,
    const float mountingAngleOffset)
    : m_totalLidarDataPoints(number_of_lidar_data_points),
      m_scanDistance(scan_distance),
      m_angleRange_Radians{angle_range_Radians},
      m_detectionThreshold(detection_threshold),
      m_anglePerRayIncrement_Radians((m_angleRange_Radians.max - m_angleRange_Radians.min) / m_totalLidarDataPoints),
      m_transformation_to_robot_frame(transformation_to_robot_frame),
      m_mountingAngleOffset(mountingAngleOffset) {
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
    std::vector<obstacle_location_t> all_obstacles; //All the Obstacles in the scan

    for (std::size_t idx = 0; idx < data.size(); idx++) {
        if (derivative_data.at(idx) < 0.0F) {
            //Low Trigger Detected for Obstacle
            std::vector<obstacle_location_t> obstacles_in_derivative; //Obstacle Indexes in the current derivative
            while (idx < data.size() && derivative_data.at(idx) <= 0) {
                if (data.at(idx) > m_scanDistance.min) {
                    obstacle_location_t obstacle{.ray_index = idx, .depth = data.at(idx)};
                    obstacles_in_derivative.push_back(obstacle);
                }
                idx++;
            }

            if (idx == data.size())
                break;
            // if (derivative_data.at(idx) > 0) {
            //Obstacle Reading Complete
            for (obstacle_location_t obstacle: obstacles_in_derivative)
                all_obstacles.push_back(obstacle);
            // } else {
            //     //Detected Another Obstacle in front of current one
            // }
        }
    }
    return all_obstacles;
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::convert_obstacle_to_coordinate_in_lidar_frame(
    const std::vector<obstacle_location_t> &obstacles,
    const pose_t currentPose) const {
    std::vector<coordinate_t> obstacle_positions;
    for (std::size_t idx = 0; idx < obstacles.size(); idx++) {
        auto obstacle = obstacles.at(idx);
        float ray_id = obstacle.ray_index;
        float ray_value = obstacle.depth;
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
            positions.emplace_back(this->m_convert_ray_to_position(currentPose, ray_id, ray_value));
        }
    }

    return positions;
}


[[nodiscard]] std::tuple<std::vector<coordinate_t>, std::vector<coordinate_t>, std::vector<coordinate_t> >
lidar_data::process_lidar_scan_to_robot_frame(
    const std::vector<float> &lidar_scan,
    const pose_t current_pose) {
    const auto derivative = this->calculate_derivative(lidar_scan);

    const auto obstacles =
            this->find_obstacles(
                lidar_scan,
                derivative);

    auto lidar_map_lidar_frame =
            this->convert_scan_to_coordinate_in_lidar_frame(
                lidar_scan,
                current_pose);

    auto obstacles_coordinates_lidar_frame =
            this->convert_obstacle_to_coordinate_in_lidar_frame(
                obstacles,
                current_pose);

    auto lidar_map_robot_frame =
            this->transform_from_lidar_frame_to_robot_frame(lidar_map_lidar_frame);

    auto obstacles_coordinates_robot_frame =
            this->transform_from_lidar_frame_to_robot_frame(
                obstacles_coordinates_lidar_frame);

    return std::make_tuple<std::vector<coordinate_t>, std::vector<coordinate_t>, std::vector<coordinate_t> >(
        std::move(lidar_map_robot_frame),
        std::move(obstacles_coordinates_robot_frame),
        std::move(obstacles_coordinates_lidar_frame));
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::get_coordinates_in_robot_origin_frame(
    const std::vector<coordinate_t> local_frame_coordinates,
    const pose_t current_pose) {
    std::vector<coordinate_t> robot_frame_coordinates;
    for (auto [x_coord, y_coord]: local_frame_coordinates) {
        if (x_coord != -1.0F) {
            // Apply rotation if needed
            float theta = current_pose.theta;
            float x = current_pose.x + (x_coord * std::cos(theta) - y_coord * sin(theta));
            float y = current_pose.y + (x_coord * sin(theta) + y_coord * cos(theta));
            robot_frame_coordinates.emplace_back(coordinate_t{.x = x, .y = y});
        }
    }
    return robot_frame_coordinates;
}

[[nodiscard]] std::vector<coordinate_t> lidar_data::transform_from_lidar_frame_to_robot_frame(
    const std::vector<coordinate_t> local_frame_coordinates) {
    std::vector<coordinate_t> robot_frame_coordinates;
    for (auto [x_coord, y_coord]: local_frame_coordinates) {
        if (x_coord != -1.0F) {
            // Apply rotation if needed
            float theta = m_transformation_to_robot_frame.theta;
            float x = m_transformation_to_robot_frame.x + (x_coord * std::cos(theta) - y_coord * sin(theta));
            float y = m_transformation_to_robot_frame.y + (x_coord * sin(theta) + y_coord * cos(theta));
            robot_frame_coordinates.emplace_back(coordinate_t{.x = x, .y = y});
        }
    }
    return robot_frame_coordinates;
}


coordinate_t lidar_data::m_convert_ray_to_position(pose_t current_pose, float ray_id, float ray_value) const {
    // const float alpha = ray_id * m_anglePerRayIncrement_Radians; // Angle of the current ray
    // static const float half_FoV = (static_cast<float>(m_totalLidarDataPoints) / 2.0F) * m_anglePerRayIncrement_Radians;
    // const float gamma = half_FoV - alpha; // Adjusting alpha to the lidar frame
    const float gamma = (ray_id - 330.0) * 0.006135923151543 + m_mountingAngleOffset;
    //TODO: Change this to actual formula when you know about lidar

    const float dX = ray_value * std::cos(gamma);
    const float dY = ray_value * std::sin(gamma);

    return coordinate_t{dX, dY};
}


// Constructor for robot_odometry
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

// Destructor for robot_odometry
robot_odometry::~robot_odometry() = default;

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


//////////////////////////////////// Localization ////////////////////////////////////////////

std::vector<float> calculateDistances(const std::vector<coordinate_t> &obstacles, float grid_size);

// Calculate distances between obstacles
std::vector<float> calculateDistances(const std::vector<coordinate_t> &obstacles, float grid_size) {
    std::vector<float> distances;

    for (size_t i = 0; i < obstacles.size(); ++i) {
        for (size_t j = i + 1; j < obstacles.size(); ++j) {
            float dx = (obstacles[j].x - obstacles[i].x) / grid_size;
            float dy = (obstacles[j].y - obstacles[i].y) / grid_size;
            float distance = std::sqrt(dx * dx + dy * dy);
            distances.push_back(distance);
        }
    }

    return distances;
}

// Localize and estimate pose
pose_t localize_and_estimate_pose(const Grid &map, lidar_data *lidarObject,
                                  const std::vector<std::vector<float>> &lidar_scans,
                                  const float grid_size) {
    static pose_t origin_pose = {0, 0, 0};
    std::vector<pose_t> possible_poses;
    std::mutex poses_mutex;

    auto process_scan = [&](const std::vector<float> &lidar_scan) {
        const auto [_, obstacles_robot_frame, obstacle_lidar_frame] =
                lidarObject->process_lidar_scan_to_robot_frame(lidar_scan, origin_pose);

        auto distances = calculateDistances(obstacle_lidar_frame, grid_size);

        for (size_t x = 0; x < map.rows(); x++) {
            for (size_t y = 0; y < map.cols(); y++) {
                if (map(y, x) != cellState::FILLED) {
                    continue;
                }

                for (float theta = -M_PI; theta < M_PI; theta += 0.1f) {
                    pose_t test_pose = {x * grid_size, y * grid_size, theta};
                    bool match_found = true;

                    std::vector<coordinate_t> transformed_obstacles;
                    for (const auto &obstacle : obstacle_lidar_frame) {
                        float transformed_x = test_pose.x + obstacle.x * cos(test_pose.theta) - obstacle.y * sin(test_pose.theta);
                        float transformed_y = test_pose.y + obstacle.x * sin(test_pose.theta) + obstacle.y * cos(test_pose.theta);
                        transformed_obstacles.push_back({transformed_x, transformed_y});
                    }

                    auto transformed_distances = calculateDistances(transformed_obstacles, grid_size);

                    if (transformed_distances.size() != distances.size()) {
                        continue;
                    }

                    for (size_t i = 0; i < distances.size(); ++i) {
                        if (std::abs(distances[i] - transformed_distances[i]) > 0.1) {
                            match_found = false;
                            break;
                        }
                    }

                    if (match_found) {
                        std::lock_guard<std::mutex> lock(poses_mutex);
                        possible_poses.push_back(test_pose);
                    }
                }
            }
        }
    };

    std::vector<std::thread> threads;
    for (const auto &lidar_scan : lidar_scans) {
        threads.emplace_back(process_scan, std::ref(lidar_scan));
    }

    for (auto &thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    if (possible_poses.empty()) {
        return {0, 0, 0};
    }

    return possible_poses[0];
}
