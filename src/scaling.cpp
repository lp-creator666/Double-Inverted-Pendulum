#include "scaling.hpp"

#include <cmath>
#include <stdexcept>

ScaledLinearization scale_linearization(
    const MatrixA& A,
    const VectorB& B,
    const Scaling& scaling
) {
    if (!std::isfinite(scaling.input_scale) ||
        scaling.input_scale <= 0.0) {
        throw std::invalid_argument("输入尺度必须为正数且有限");
    }

    for (double value : scaling.state_scale) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument("状态尺度必须为正数且有限");
        }
    }

    for (double value : A) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("矩阵 A 包含非有限值");
        }
    }
    for (double value : B) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("向量 B 包含非有限值");
        }
    }

    ScaledLinearization result;

    // z = Sx * x, v = u / input_scale。
    // As = Sx * A * Sx^{-1}。
    for (int row = 0; row < kStateDimension; ++row) {
        for (int column = 0; column < kStateDimension; ++column) {
            result.A[row * kStateDimension + column] =
                A[row * kStateDimension + column] *
                scaling.state_scale[column] /
                scaling.state_scale[row];
        }
    }

    // Bs = Sx * B * input_scale。
    for (int row = 0; row < kStateDimension; ++row) {
        result.B[row] =
            B[row] * scaling.input_scale / scaling.state_scale[row];
    }

    return result;
}
