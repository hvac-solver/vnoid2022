#pragma once

#include <stdexcept>
#include <iostream>
#include <yaml-cpp/yaml.h>

struct MPCParams 
{
    double T = 0.7;
    int N = 20;
    int nthreads = 6;
    int iter = 1;
    int sim_steps_per_mpc_update = 1;

    void loadFromYAML(const YAML::Node& config) {
        if (config["T"].IsDefined()) 
            T = config["T"].as<double>();
        if (config["N"].IsDefined()) 
            N = config["N"].as<int>();
        if (config["iter"].IsDefined()) 
            iter = config["iter"].as<int>();
        if (config["nthreads"].IsDefined()) 
            nthreads = config["nthreads"].as<int>();
        if (config["sim_steps_per_mpc_update"].IsDefined()) 
            sim_steps_per_mpc_update = config["sim_steps_per_mpc_update"].as<int>();
    }

    void check() const {
        if (T <= 0.0) {
            throw std::invalid_argument("MPCParams.T must be positive!");
        }
        if (N < 0.0) {
            throw std::invalid_argument("MPCParams.N must be positive!");
        }
        if (nthreads < 0.0) {
            throw std::invalid_argument("MPCParams.nthreads must be positive!");
        }
        if (iter < 0.0) {
            throw std::invalid_argument("MPCParams.iter must be positive!");
        }
        if (sim_steps_per_mpc_update < 0.0) {
            throw std::invalid_argument("MPCParams.sim_steps_per_mpc_update must be positive!");
        }
    }

    friend std::ostream& operator<<(std::ostream& os, 
                                    const MPCParams& mpc_params) {
        os << "MPCParams:" << "\n";
        os << "  T:                          " << mpc_params.T << "\n";
        os << "  N:                          " << mpc_params.N << "\n";
        os << "  nthreads:                   " << mpc_params.nthreads << "\n";
        os << "  iter:                       " << mpc_params.iter << "\n";
        os << "  sim_steps_per_mpc_update:   " << mpc_params.sim_steps_per_mpc_update << "\n";
        return os;
    }
};