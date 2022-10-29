#pragma once

#include "Eigen/Core"

#include <stdexcept>
#include <iostream>
#include <yaml-cpp/yaml.h>

struct StairClimbingParams 
{
    double knee_angle = M_PI / 3.0;

    Eigen::Vector3d stair_step_length = {0.3, 0.0, 0.2}; 
    Eigen::Vector3d floor_step_length = {0.45, 0.0, 0.0}; 

    double step_height = 0.4;
    double swing_time = 0.5;
    double double_support_time = 0.0; // must be zero with the current StairClimbingFootStepPlanner implementation.
    double swing_start_time = 0.5;
    double height_offset = 0.05;

    int num_stair_steps = 14;
    int num_floor_steps = 3 + 2; // 2 is offset

    double initial_time = 0.0;
    Eigen::Vector3d initial_base_position = {5.35, 1.5, 0.75}; 

    void loadFromYAML(const YAML::Node& config) {
        if (config["knee_angle"].IsDefined()) 
            knee_angle = config["knee_angle"].as<double>();
        if (config["stair_step_length"].IsDefined()) {
            const auto stair_step_length_stdvec = config["stair_step_length"].as<std::vector<double>>();
            if (stair_step_length_stdvec.size() != 3) {
                throw std::invalid_argument("[FastWalkingParams::loadFromYAML] stair_step_length.size() must be 3!");
            }
            stair_step_length = Eigen::Map<const Eigen::Vector3d>(stair_step_length_stdvec.data());
        }
        if (config["floor_step_length"].IsDefined()) {
            const auto floor_step_length_stdvec = config["floor_step_length"].as<std::vector<double>>();
            if (floor_step_length_stdvec.size() != 3) {
                throw std::invalid_argument("[FastWalkingParams::loadFromYAML] floor_step_length.size() must be 3!");
            }
            floor_step_length = Eigen::Map<const Eigen::Vector3d>(floor_step_length_stdvec.data());
        }
        if (config["step_height"].IsDefined()) 
            step_height = config["step_height"].as<double>();
        if (config["swing_time"].IsDefined()) 
            swing_time = config["swing_time"].as<double>();
        if (config["swing_start_time"].IsDefined()) 
            swing_start_time = config["swing_start_time"].as<double>();
        if (config["height_offset"].IsDefined()) 
            height_offset = config["height_offset"].as<double>();
        if (config["initial_time"].IsDefined()) 
            initial_time = config["initial_time"].as<double>();
        if (config["initial_base_position"].IsDefined()) {
            const auto initial_base_position_stdvec = config["initial_base_position"].as<std::vector<double>>();
            if (initial_base_position_stdvec.size() != 3) {
                throw std::invalid_argument("[FastWalkingParams::loadFromYAML] initial_base_position.size() must be 3!");
            }
            initial_base_position = Eigen::Map<const Eigen::Vector3d>(initial_base_position_stdvec.data());
        }
    }

    void check() const {
        if (knee_angle < 0.0) {
            throw std::invalid_argument("StairClimbingParams.knee_angle must be non-negative!");
        }
        if (step_height <= 0.0) {
            throw std::invalid_argument("StairClimbingParams.step_height must be positive!");
        }
        if (swing_time <= 0.0) {
            throw std::invalid_argument("StairClimbingParams.swing_time must be positive!");
        }
        if (double_support_time < 0.0) {
            throw std::invalid_argument("StairClimbingParams.double_support_time must be non-negative!");
        }
        if (swing_start_time - initial_time <= 0.0) {
            throw std::invalid_argument("StairClimbingParams.swing_start_time - StairClimbingParams.initial_time must be positive!");
        }
        if (height_offset < 0.0) {
            throw std::invalid_argument("StairClimbingParams.height_offset must be non-negative!");
        }
        if (num_stair_steps <= 0) {
            throw std::invalid_argument("StairClimbingParams.num_stair_steps must be positive!");
        }
    }

    friend std::ostream& operator<<(std::ostream& os, 
                                    const StairClimbingParams& climbing_params) {
        os << "StairClimbingParams: " << "\n";
        os << "  knee_angle:            " << climbing_params.knee_angle << "\n";
        os << "  stair_step_length:     " << climbing_params.stair_step_length.transpose() << "\n";
        os << "  floor_step_length:     " << climbing_params.floor_step_length.transpose() << "\n";
        os << "  step_height:           " << climbing_params.step_height << "\n";
        os << "  swing_time:            " << climbing_params.swing_time << "\n";
        os << "  double_support_time:   " << climbing_params.double_support_time << "\n";
        os << "  swing_start_time:      " << climbing_params.swing_start_time << "\n";
        os << "  height_offset:         " << climbing_params.height_offset << "\n";
        os << "  num_stair_steps:       " << climbing_params.num_stair_steps << "\n";
        os << "  num_floor_steps:       " << climbing_params.num_floor_steps << "\n";
        os << "  initial_time:          " << climbing_params.initial_time << "\n";
        os << "  initial_base_position: " << climbing_params.initial_base_position.transpose() << "\n";
        return os;
    }
};