#pragma once

#include "logging.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <numeric>
#include <random>
#include <vector>

namespace erl::common {

    extern std::mt19937 g_random_engine;

    template<typename T>
    std::vector<T>
    GenerateShuffledIndices(std::size_t size) {
        std::vector<T> indices(size);
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), g_random_engine);
        return indices;
    }

    template<typename T>
    std::vector<T>
    GenerateShuffledIndices(std::size_t size, std::mt19937 &rd) {
        std::vector<T> indices(size);
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), rd);
        return indices;
    }

    template<typename T>
    std::vector<T>
    GenerateShuffledIndices(std::size_t num_samples, double ratio) {
        std::vector<T> indices(num_samples);
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), g_random_engine);
        ERL_ASSERTM(ratio > 0.0 && ratio <= 1.0, "ratio must be in (0.0, 1.0]");
        if (ratio < 1.0) {
            num_samples = static_cast<std::size_t>(std::ceil(static_cast<double>(num_samples) * ratio));
            indices.resize(num_samples);
        }
        return indices;
    }

    inline Eigen::VectorXd
    GenerateGaussianNoise(long size, double mean, double scale) {
        Eigen::VectorXd noise(size);
        std::normal_distribution<double> distribution(mean, scale);
        for (long i = 0; i < size; ++i) { noise[i] = distribution(common::g_random_engine); }
        return noise;
    }

    inline Eigen::MatrixXd
    GenerateGaussianNoise(long rows, long cols, double mean, double scale) {
        Eigen::MatrixXd noise(rows, cols);
        std::normal_distribution<double> distribution(mean, scale);
        for (long i = 0; i < rows; ++i) {
            for (long j = 0; j < cols; ++j) { noise(i, j) = distribution(common::g_random_engine); }
        }
        return noise;
    }

    inline void
    ManuallySetSeed(unsigned int seed) {
        g_random_engine.seed(seed);
    }
}  // namespace erl::common
