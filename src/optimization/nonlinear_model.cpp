#include "optimization/nonlinear_model.hpp"
#include <cmath>
#include <stdexcept>
#include "model/dynamics.hpp"

NmpcPrediction predict_nonlinear_horizon(
    const mjModel* model,
    mjData* prediction_data,
    const State& initial_state,
    const int* qpos_addresses,
    const int* qvel_addresses,
    const std::vector<double>& forces,
    int steps_per_control
) {
    if (model == nullptr || prediction_data == nullptr) {
        throw std::invalid_argument("Prediction model or data is null");
    }

    if (qpos_addresses == nullptr || qvel_addresses == nullptr) {
        throw std::invalid_argument("Joint addresses are null");
    }

    for (int i = 0; i < 3; ++i) {
        if (qpos_addresses[i] < 0 || qpos_addresses[i] >= model->nq ||
            qvel_addresses[i] < 0 || qvel_addresses[i] >= model->nv) {
            throw std::invalid_argument("Joint address is out of range");
        }
    }

    if (!initial_state.finite()) {
        throw std::invalid_argument("Initial state is not finite");
    }

    if (steps_per_control <= 0 ||
        !std::isfinite(model->opt.timestep) ||
        model->opt.timestep <= 0.0) {
        throw std::invalid_argument("Invalid prediction timestep");
    }

    if (forces.empty()) {
        throw std::invalid_argument("Force sequence is empty");
    }

    if (model->nu != 1 || !model->actuator_ctrllimited[0]) {
        throw std::invalid_argument(
            "Expected one actuator with a control limit"
        );
    }

    const double force_min = model->actuator_ctrlrange[0];
    const double force_max = model->actuator_ctrlrange[1];

    if (!std::isfinite(force_min) || !std::isfinite(force_max) ||
        force_min >= force_max) {
        throw std::invalid_argument("Invalid actuator control range");
    }

    for (double force : forces) {
        if (!std::isfinite(force) ||
            force < force_min || force > force_max) {
            throw std::invalid_argument(
                "Prediction force is non-finite or out of range"
            );
        }
    }
    NmpcPrediction prediction;
    prediction.control_dt =
        model->opt.timestep * steps_per_control;

    if (!std::isfinite(prediction.control_dt)) {
        throw std::invalid_argument("Control timestep is not finite");
    }

    prediction.states.reserve(forces.size() + 1);
    prediction.states.push_back(initial_state);

    State current_state = initial_state;

    for (double force : forces) {
        State next_state = predict_state(
            model,
            prediction_data,
            current_state,
            qpos_addresses,
            qvel_addresses,
            force,
            steps_per_control
        );

        if (!next_state.finite()) {
            throw std::runtime_error(
                "Nonlinear prediction produced a non-finite state"
            );
        }

        for (int i = 0; i < mjNWARNING; ++i) {
            if (prediction_data->warning[i].number != 0) {
                throw std::runtime_error(
                    "MuJoCo warning during nonlinear prediction"
                );
            }
        }

        prediction.states.push_back(next_state);
        current_state = next_state;
    }

    return prediction;
}

namespace {
State add_scaled(const State& state, int index, double amount) {
    State result = state;
    double* values[] = {&result.p, &result.dp, &result.q1,
                        &result.dq1, &result.q2, &result.dq2};
    *values[index] += amount;
    return result;
}

State subtract(const State& lhs, const State& rhs) {
    return {lhs.p-rhs.p, lhs.dp-rhs.dp, lhs.q1-rhs.q1,
            lhs.dq1-rhs.dq1, lhs.q2-rhs.q2, lhs.dq2-rhs.dq2};
}

void set_column(MatrixA& matrix, int column, const State& value,
                double denominator) {
    const double values[] = {value.p, value.dp, value.q1,
                             value.dq1, value.q2, value.dq2};
    for (int row = 0; row < kStateDimension; ++row) {
        matrix[row * kStateDimension + column] = values[row] / denominator;
    }
}

void validate_step(const mjModel* model, int steps, double state_eps,
                   double input_eps) {
    if (model == nullptr || steps <= 0 || !std::isfinite(model->opt.timestep) ||
        model->opt.timestep <= 0.0 || !std::isfinite(state_eps) ||
        state_eps <= 0.0 || !std::isfinite(input_eps) || input_eps <= 0.0) {
        throw std::invalid_argument("Invalid NMPC linearization settings");
    }
}
}

NmpcTrajectoryLinearization linearize_nmpc_trajectory(
    const mjModel* model,
    mjData* prediction_data,
    const std::vector<State>& candidate_states,
    const std::vector<double>& forces,
    const int* qpos_addresses,
    const int* qvel_addresses,
    int steps_per_control,
    double state_epsilon,
    double input_epsilon
) {
    if (prediction_data == nullptr || qpos_addresses == nullptr ||
        qvel_addresses == nullptr) {
        throw std::invalid_argument("Null NMPC linearization input");
    }
    validate_step(model, steps_per_control, state_epsilon, input_epsilon);
    if(model->nu!=1 || !model->actuator_ctrllimited[0])
        throw std::invalid_argument("Expected one bounded actuator");
    for(int i=0;i<3;++i) if(qpos_addresses[i]<0 || qpos_addresses[i]>=model->nq ||
        qvel_addresses[i]<0 || qvel_addresses[i]>=model->nv)
        throw std::invalid_argument("Linearization joint address out of range");
    if (candidate_states.size() != forces.size() + 1 || forces.empty()) {
        throw std::invalid_argument("NMPC linearization sizes do not match");
    }
    for (const State& state : candidate_states) {
        if (!state.finite()) throw std::invalid_argument("Non-finite candidate state");
    }
    NmpcTrajectoryLinearization result;
    result.state_epsilon = state_epsilon;
    result.input_epsilon = input_epsilon;
    result.segments.reserve(forces.size());

    for (std::size_t k = 0; k < forces.size(); ++k) {
        const State& x = candidate_states[k];
        const double u = forces[k];
        if(!std::isfinite(u) || u<model->actuator_ctrlrange[0] || u>model->actuator_ctrlrange[1])
            throw std::invalid_argument("Input outside actuator range");
        const State base = predict_state(model, prediction_data, x,
                                         qpos_addresses, qvel_addresses,
                                         u, steps_per_control);
        if (!base.finite()) throw std::runtime_error("Non-finite base prediction");
        NmpcSegmentLinearization segment;
        segment.predicted_next = base;
        for (int column = 0; column < kStateDimension; ++column) {
            const State plus = predict_state(model, prediction_data,
                add_scaled(x, column, state_epsilon), qpos_addresses,
                qvel_addresses, u, steps_per_control);
            const State minus = predict_state(model, prediction_data,
                add_scaled(x, column, -state_epsilon), qpos_addresses,
                qvel_addresses, u, steps_per_control);
            set_column(segment.A, column, subtract(plus, minus),
                       2.0 * state_epsilon);
        }
        const double lower = model->actuator_ctrlrange[0];
        const double upper = model->actuator_ctrlrange[1];
        if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper ||
            u < lower || u > upper) {
            throw std::invalid_argument("Input outside actuator range");
        }
        const double h=std::min(input_epsilon,(upper-lower)/2);
        const double plus_step = upper-u<h?0:h;
        const double minus_step = u-lower<h?0:h;
        if (plus_step <= 0.0 && minus_step <= 0.0) {
            throw std::invalid_argument("No valid input perturbation");
        }
        if (plus_step > 0.0 && minus_step > 0.0) {
            const State plus_u = predict_state(model, prediction_data, x,
                qpos_addresses, qvel_addresses, u + plus_step, steps_per_control);
            const State minus_u = predict_state(model, prediction_data, x,
                qpos_addresses, qvel_addresses, u - minus_step, steps_per_control);
            const State diff_u = subtract(plus_u, minus_u);
            const double values[] = {diff_u.p, diff_u.dp, diff_u.q1,
                                     diff_u.dq1, diff_u.q2, diff_u.dq2};
            const double denominator = plus_step + minus_step;
            for (int row = 0; row < kStateDimension; ++row) {
                segment.B[row] = values[row] / denominator;
            }
        } else if (plus_step > 0.0) {
            const State plus_u = predict_state(model, prediction_data, x,
                qpos_addresses, qvel_addresses, u + plus_step, steps_per_control);
            const State diff_u = subtract(plus_u, base);
            const double values[] = {diff_u.p, diff_u.dp, diff_u.q1,
                                     diff_u.dq1, diff_u.q2, diff_u.dq2};
            for (int row = 0; row < kStateDimension; ++row) {
                segment.B[row] = values[row] / plus_step;
            }
        } else {
            const State minus_u = predict_state(model, prediction_data, x,
                qpos_addresses, qvel_addresses, u - minus_step, steps_per_control);
            const State diff_u = subtract(base, minus_u);
            const double values[] = {diff_u.p, diff_u.dp, diff_u.q1,
                                     diff_u.dq1, diff_u.q2, diff_u.dq2};
            for (int row = 0; row < kStateDimension; ++row) {
                segment.B[row] = values[row] / minus_step;
            }
        }
        for (double value : segment.A) {
            if (!std::isfinite(value)) throw std::runtime_error("Non-finite A");
        }
        for (double value : segment.B) {
            if (!std::isfinite(value)) throw std::runtime_error("Non-finite B");
        }
        result.segments.push_back(segment);
    }
    return result;
}
