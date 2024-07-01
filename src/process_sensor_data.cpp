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

lidar_data ::lidar_data(
    std::size_t number_of_lidar_data_points,
    value_range_t scan_distance,
    value_range_t angle_range)
    : m_totalLidarDataPoints(number_of_lidar_data_points),
      m_scanDistance(scan_distance),
      m_angleRange{angle_range}
{
}

lidar_data ::~lidar_data()
{
}

std::vector<float> lidar_data::m_process_data(std::vector<float> &data)
{
    std::vector<float> processed_data;
    processed_data.reserve(this->m_totalLidarDataPoints); // Reserve space to avoid reallocations

    for (size_t i = 0; i < this->m_totalLidarDataPoints; i++)
    {
        float val = data.at(i);
        if (val >= this->m_scanDistance.max)
        {
            val = std::numeric_limits<float>::max();
        }
        else if (val <= this->m_scanDistance.min)
        {
            val = std::numeric_limits<float>::min();
        }
        processed_data.emplace_back(val);
    }

    return std::move(processed_data);
}

robot_odometry::robot_odometry(
    const std::uint32_t encoder_ticks_per_revolution,
    const float wheel_radius_in_mm,
    const float distance_between_wheels_in_mm)
    : m_encoderCountPerRevolution(encoder_ticks_per_revolution),
      m_wheelRadius_millimeters(wheel_radius_in_mm),
      m_robotWidthBetweenWheels_millimeters(distance_between_wheels_in_mm)
{
}

robot_odometry::~robot_odometry()
{
}

void robot_odometry::m_update_pose(const encoder_ticks_t new_encoder_ticks)
{
    static float wheelCircumference = 2.0F * M_PI * this->m_wheelRadius_millimeters;

    float eR_L = (new_encoder_ticks.left / this->m_encoderCountPerRevolution) * wheelCircumference;
    float eR_R = (new_encoder_ticks.right / this->m_encoderCountPerRevolution) * wheelCircumference;

    this->m_update_pose(encoder_rot_t{.left_wheel_val = eR_L, .right_wheel_val = eR_R});
}

void robot_odometry::m_update_pose(const encoder_rot_t new_absolute_encoder_value)
{
    float dL = new_absolute_encoder_value.left_wheel_val - this->m_currentEncoderValue.left_wheel_val;
    float dR = new_absolute_encoder_value.right_wheel_val - this->m_currentEncoderValue.right_wheel_val;

    this->m_currentEncoderValue.left_wheel_val = new_absolute_encoder_value.left_wheel_val;
    this->m_currentEncoderValue.right_wheel_val = new_absolute_encoder_value.right_wheel_val;
    this->m_robotPose = this->m_calculate_motion(dL, dR);
}

/**
 * @brief
 *
 * @param x
 * @param y
 * @param theta
 */
void robot_odometry::m_update_pose(const float x, const float y, const float theta)
{
    this->m_robotPose.x = x;
    this->m_robotPose.y = y;
    this->m_robotPose.theta = theta;
}

pose_t robot_odometry::m_calculate_motion(float dL, float dR)
{
    float newTheta{0.0F}, newX{0.0F}, newY{0.0F};
    pose_t new_pose;

    if (dL = dR) // Motion is translational
    {
        newX = this->m_robotPose.x + (dL * std::cos(this->m_robotPose.theta));
        newY = this->m_robotPose.y + (dL * std::sin(this->m_robotPose.theta));
        newTheta = this->m_robotPose.theta;
    }
    else // Motion is Rotational
    {
        float alpha = (dR - dL) / this->m_robotWidthBetweenWheels_millimeters;
        float radiusOfRotation = dL / alpha;
        float RDistance = radiusOfRotation + (this->m_robotWidthBetweenWheels_millimeters / 2.0F);

        float cX = this->m_robotPose.x - (RDistance * std::sin(this->m_robotPose.theta));
        float cY = this->m_robotPose.y + (RDistance * std::cos(this->m_robotPose.theta));

        newTheta = std::fmod((this->m_robotPose.theta + alpha), (2.0F * M_PI));
        newX = cX + (RDistance * std::sin(newTheta));
        newY = cY - (RDistance * std::cos(newTheta));
    }

    new_pose.x = newX;
    new_pose.y = newY;
    new_pose.theta = newTheta;
    return std::move(new_pose);
}
