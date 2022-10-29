#pragma once

#include "Eigen/Core"

#include <stdexcept>
#include <iostream>
#include <yaml-cpp/yaml.h>

struct FastWalkingParams 
{
    double knee_angle = M_PI / 6.0;

    Eigen::Vector3d step_length = {0.75, 0.0, 0.0}; 
    double step_yaw = 0.0;

    double step_height = 0.15;
    double swing_time = 0.6;
    double double_support_time = 0.0;
    double swing_start_time = 0.5;
    double height_offset = 0.05;

    bool use_raibert = false;
    double raibert_gain = 0.7;

    void loadFromYAML(const YAML::Node& config) {
        if (config["knee_angle"].IsDefined()) 
            knee_angle = config["knee_angle"].as<double>();
        if (config["step_length"].IsDefined()) {
            const auto step_length_stdvec = config["step_length"].as<std::vector<double>>();
            if (step_length_stdvec.size() != 3) {
                throw std::invalid_argument("[FastWalkingParams::loadFromYAML] step_length.size() must be 3!");
            }
            step_length = Eigen::Map<const Eigen::Vector3d>(step_length_stdvec.data());
        }
        if (config["step_yaw"].IsDefined()) 
            step_yaw = config["step_yaw"].as<double>();
        if (config["step_height"].IsDefined()) 
            step_height = config["step_height"].as<double>();
        if (config["swing_time"].IsDefined()) 
            swing_time = config["swing_time"].as<double>();
        if (config["double_support_time"].IsDefined()) 
            double_support_time = config["double_support_time"].as<double>();
        if (config["swing_start_time"].IsDefined()) 
            swing_start_time = config["swing_start_time"].as<double>();
        if (config["height_offset"].IsDefined()) 
            height_offset = config["height_offset"].as<double>();
        if (config["use_raibert"].IsDefined()) 
            use_raibert = config["use_raibert"].as<bool>();
        if (config["raibert_gain"].IsDefined()) 
            raibert_gain = config["raibert_gain"].as<double>();
    }

    void check() const {
        if (knee_angle < 0.0) {
            throw std::invalid_argument("FastWalkingParams.knee_angle must be non-negative!");
        }
        if (step_height <= 0.0) {
            throw std::invalid_argument("FastWalkingParams.step_height must be positive!");
        }
        if (swing_time <= 0.0) {
            throw std::invalid_argument("FastWalkingParams.swing_time must be positive!");
        }
        if (double_support_time < 0.0) {
            throw std::invalid_argument("FastWalkingParams.double_support_time must be non-negative!");
        }
        if (swing_start_time <= 0.0) {
            throw std::invalid_argument("FastWalkingParams.swing_start_time must be positive!");
        }
        if (height_offset < 0.0) {
            throw std::invalid_argument("FastWalkingParams.height_offset must be non-negative!");
        }
        if (use_raibert && raibert_gain <= 0.0) {
            throw std::invalid_argument("FastWalkingParams.raibert_gain must be positive!");
        }
    }

    friend std::ostream& operator<<(std::ostream& os, 
                                    const FastWalkingParams& walking_params) {
        os << "FastWalkingParams: " << "\n";
        os << "  knee_angle:          " << walking_params.knee_angle << "\n";
        os << "  step_length:         " << walking_params.step_length.transpose() << "\n";
        os << "  step_yaw:            " << walking_params.step_yaw << "\n";
        os << "  step_height:         " << walking_params.step_height << "\n";
        os << "  swing_time:          " << walking_params.swing_time << "\n";
        os << "  double_support_time: " << walking_params.double_support_time << "\n";
        os << "  swing_start_time:    " << walking_params.swing_start_time << "\n";
        os << "  height_offset:       " << walking_params.height_offset << "\n";
        os << "  use_raibert:         " << std::boolalpha << walking_params.use_raibert << "\n";
        os << "  raibert_gain:        " << walking_params.raibert_gain << "\n";
        return os;
    }
};