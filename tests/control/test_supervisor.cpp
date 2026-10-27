#include "control/supervisor.hpp"

#include <iostream>
#include <cmath>
#include <stdexcept>

int main() try {
    Supervisor supervisor;
    supervisor.start_swingup();
    State outside{0, 0, 0.15, 0, 0, 0};
    if (supervisor.update(outside, true) != ControlStage::SwingUp)
        throw std::runtime_error("left swing-up too early");
    supervisor = Supervisor();
    if (supervisor.update(outside, true) != ControlStage::Sqp)
        throw std::runtime_error("entered capture too early");
    State inside{0, 0, 0.05, 0, 0, 0};
    if (supervisor.update(inside, true) != ControlStage::Lqr)
        throw std::runtime_error("did not enter capture");
    State hysteresis{0, 0, 0.15, 0, 0, 0};
    if (supervisor.update(hysteresis, true) != ControlStage::Lqr)
        throw std::runtime_error("capture hysteresis failed");
    State leave{0, 0, 0.19, 0, 0, 0};
    if (supervisor.update(leave, true) != ControlStage::Sqp)
        throw std::runtime_error("did not leave capture");
    if (supervisor.update(leave, false) != ControlStage::Fault)
        throw std::runtime_error("SQP failure was not latched");
    Supervisor periodic;
    State rotated{}; rotated.q1=2*std::acos(-1.0)+0.01;
    if (periodic.update(rotated,true)!=ControlStage::Lqr ||
        std::abs(local_balance_state(rotated).q1-0.01)>1e-12)
        throw std::runtime_error("Periodic capture failed");
    State invalid; invalid.q1 = 0.0 / 0.0;
    if (supervisor.update(invalid, true) != ControlStage::Fault)
        throw std::runtime_error("invalid state was not faulted");
    std::cout << "Supervisor hysteresis checks passed\n";
    return 0;
} catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
}
