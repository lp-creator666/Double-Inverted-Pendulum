#pragma once

#include <cmath>

struct State
{
    double p = 0.0;
    double dp = 0.0;
    
    double q1 = 0.0;
    double dq1 = 0.0;
    double q2 = 0.0;
    double dq2 = 0.0;

    bool finite() const {
        return std::isfinite(p) && std::isfinite(dp) &&
               std::isfinite(q1) && std::isfinite(dq1) &&
               std::isfinite(q2) && std::isfinite(dq2);
    }
};
