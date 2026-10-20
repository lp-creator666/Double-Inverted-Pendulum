#include "logger.hpp"

#include <filesystem>
#include <iomanip>
#include <stdexcept>

CsvLogger::CsvLogger(const std::string& path) {
    const std::filesystem::path output(path);
    if (output.has_parent_path()) {
        std::filesystem::create_directories(output.parent_path());
    }
    file_.open(output, std::ios::out | std::ios::trunc);
    if (!file_) throw std::runtime_error("无法打开日志文件: " + path);
    file_ << std::setprecision(17)
          << "time,p,dp,q1,dq1,q2,dq2,pole2_absolute_angle,"
             "requested_force,applied_force\n";
}

void CsvLogger::write(double time, const State& state,
                      double requested_force, double applied_force) {
    file_ << time << ','
          << state.p << ',' << state.dp << ','
          << state.q1 << ',' << state.dq1 << ','
          << state.q2 << ',' << state.dq2 << ','
          << state.q1 + state.q2 << ','
          << requested_force << ',' << applied_force << '\n';
    if (!file_) throw std::runtime_error("写入 CSV 日志失败");
}
