#include "swingup_controller.hpp"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <functional>

int main() try {
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML("model/double_cartpole.xml", nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) throw std::runtime_error(error);
    model->opt.enableflags |= mjENBL_ENERGY;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        mj_makeData(model.get()), mj_deleteData);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> target(
        mj_makeData(model.get()), mj_deleteData);
    if (!data || !target) throw std::runtime_error("Data allocation failed");
    const int q1_id = mj_name2id(model.get(), mjOBJ_JOINT, "pole_1_hinge");
    const int q2_id = mj_name2id(model.get(), mjOBJ_JOINT, "pole_2_hinge");
    data->qpos[model->jnt_qposadr[q1_id]] = 1.5707963267948966;
    data->qpos[model->jnt_qposadr[q2_id]] = -1.0;
    mj_forward(model.get(), data.get());
    SwingupController controller(model.get(), target.get());
    const State state{0, 0, 1.5707963267948966, 0.4, -1.0, -0.2};
    const double force = controller.compute_force(model.get(), data.get(), state);
    if (!std::isfinite(force) || std::abs(force) > 20.0)
        throw std::runtime_error("Invalid swing-up force");
    if (!std::isfinite(controller.target_energy()))
        throw std::runtime_error("Invalid target energy");
    mj_forward(model.get(), target.get());
    mj_energyPos(model.get(), target.get());
    if (std::abs(controller.target_energy()-target->energy[0])>1e-10)
        throw std::runtime_error("Target energy is not model-derived");
    // The production callback outlives the block that constructs its controller.
    std::function<double()> callback;
    {
        auto owned=std::make_shared<SwingupController>(model.get(), target.get());
        callback=[owned,&model,&data,state] { return owned->compute_force(model.get(),data.get(),state); };
    }
    if (std::abs(callback()-force)>1e-12 || std::abs(force)<1e-3)
        throw std::runtime_error("Swing-up callback lifetime or excitation regression");
    std::cout << "Swing-up controller checks passed\n";
    return 0;
} catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
}
