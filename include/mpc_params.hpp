#pragma once

#include <stdexcept>

struct MPCParams 
{
    double T = 0.7;
    int N = 20;
    int nthreads = 6;
    int iter = 1;
    int sim_steps_per_mpc_update = 1;
};

void checkMPCParams(const MPCParams& mpc_params) 
{
    if (mpc_params.T <= 0.0) {
        throw std::runtime_error("MPCParams.T must be positive!");
    }
    if (mpc_params.N < 0.0) {
        throw std::runtime_error("MPCParams.N must be positive!");
    }
    if (mpc_params.nthreads < 0.0) {
        throw std::runtime_error("MPCParams.nthreads must be positive!");
    }
    if (mpc_params.iter < 0.0) {
        throw std::runtime_error("MPCParams.iter must be positive!");
    }
    if (mpc_params.sim_steps_per_mpc_update < 0.0) {
        throw std::runtime_error("MPCParams.sim_steps_per_mpc_update must be positive!");
    }
}