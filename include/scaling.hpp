#pragma once

#include "linearize.hpp"

struct Scaling {
    StateVector state_scale{};
    double input_scale = 1.0;
};

struct ScaledLinearization {
    MatrixA A{};
    VectorB B{};
};

ScaledLinearization scale_linearization(
    const MatrixA& A,
    const VectorB& B,
    const Scaling& scaling
);
