#pragma once

namespace erl::common {

    inline double
    ClipRange(const double value, const double min, const double max) {
        if (value < min) { return min; }
        if (value > max) { return max; }
        return value;
    }
}  // namespace erl::common
