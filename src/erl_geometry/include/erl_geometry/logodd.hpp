#pragma once

#include <cmath>

namespace erl::geometry::logodd {

    inline float
    LogOdd(const double p) {
        return static_cast<float>(std::log(p / (1 - p)));
    }

    inline double
    Probability(const float logodd) {
        return 1.0 / (1.0 + std::exp(-logodd));
    }
}  // namespace erl::geometry::logodd
