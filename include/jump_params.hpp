#pragma once

#include "Eigen/Core"

#include <stdexcept>
#include <iostream>
#include <yaml-cpp/yaml.h>

struct JumpParams 
{
    double knee_angle = M_PI / 3.0;

    Eigen::Vector3d jump_length = {1.5, 0.0, -0.9}; 
    Eigen::Vector3d step_length = {0.5, 0.0,  0.0}; 
    double jump_height = 0.3;

    double ground_time = 0.3;
    double flying_up_time = 0.3;
    double flying_time = 0.3;
    double landing_time = 0.3;

    double initial_time = 0.0;
    Eigen::Vector3d initial_base_position = {5.35, 1.5, 0.75}; 

    void loadFromYAML(const YAML::Node& config) {
        if (config["knee_angle"].IsDefined()) 
            knee_angle = config["knee_angle"].as<double>();
        if (config["jump_length"].IsDefined()) {
            const auto jump_length_stdvec = config["jump_length"].as<std::vector<double>>();
            if (jump_length_stdvec.size() != 3) {
                throw std::invalid_argument("[JumpParams::loadFromYAML] jump_length.size() must be 3!");
            }
            jump_length = Eigen::Map<const Eigen::Vector3d>(jump_length_stdvec.data());
        }
        if (config["step_length"].IsDefined()) {
            const auto step_length_stdvec = config["step_length"].as<std::vector<double>>();
            if (step_length_stdvec.size() != 3) {
                throw std::invalid_argument("[JumpParams::loadFromYAML] step_length.size() must be 3!");
            }
            step_length = Eigen::Map<const Eigen::Vector3d>(step_length_stdvec.data());
        }
        if (config["jump_height"].IsDefined()) 
            jump_height = config["jump_height"].as<double>();
        if (config["ground_time"].IsDefined()) 
            ground_time = config["ground_time"].as<double>();
        if (config["flying_up_time"].IsDefined()) 
            flying_up_time = config["flying_up_time"].as<double>();
        if (config["flying_time"].IsDefined()) 
            flying_time = config["flying_time"].as<double>();
        if (config["landing_time"].IsDefined()) 
            landing_time = config["landing_time"].as<double>();
        if (config["initial_time"].IsDefined()) 
            initial_time = config["initial_time"].as<double>();
        if (config["initial_base_position"].IsDefined()) {
            const auto initial_base_position_stdvec = config["initial_base_position"].as<std::vector<double>>();
            if (initial_base_position_stdvec.size() != 3) {
                throw std::invalid_argument("[JumpParams::loadFromYAML] initial_base_position.size() must be 3!");
            }
            initial_base_position = Eigen::Map<const Eigen::Vector3d>(initial_base_position_stdvec.data());
        }
    }

    void check() const {
        if (knee_angle < 0.0) {
            throw std::invalid_argument("JumpParams.knee_angle must be non-negative!");
        }
        if (jump_height < 0.0) {
            throw std::invalid_argument("JumpParams.jump_height must be non-negative!");
        }
        if (ground_time <= 0.0) {
            throw std::invalid_argument("JumpParams.ground_time must be positive!");
        }
        if (flying_up_time <= 0.0) {
            throw std::invalid_argument("JumpParams.flying_up_time must be non-negative!");
        }
        if (flying_time <= 0.0) {
            throw std::invalid_argument("JumpParams.flying_time must be positive!");
        }
        if (landing_time <= 0.0) {
            throw std::invalid_argument("JumpParams.landing_time must be non-negative!");
        }
    }

    friend std::ostream& operator<<(std::ostream& os, 
                                    const JumpParams& jump_params) {
        os << "JumpParams: " << "\n";
        os << "  knee_angle:            " << jump_params.knee_angle << "\n";
        os << "  jump_length:           " << jump_params.jump_length.transpose() << "\n";
        os << "  step_length:           " << jump_params.step_length.transpose() << "\n";
        os << "  jump_height:           " << jump_params.jump_height << "\n";
        os << "  ground_time:           " << jump_params.ground_time << "\n";
        os << "  flying_up_time:        " << jump_params.flying_up_time << "\n";
        os << "  flying_time:           " << jump_params.flying_time << "\n";
        os << "  landing_time:          " << jump_params.landing_time << "\n";
        os << "  initial_time:          " << jump_params.initial_time << "\n";
        os << "  initial_base_position: " << jump_params.initial_base_position.transpose() << "\n";
        return os;
    }
};