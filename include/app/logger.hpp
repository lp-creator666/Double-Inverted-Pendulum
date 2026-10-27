#pragma once

#include "types.hpp"

#include <fstream>
#include <string>

class CsvLogger {
public:
    explicit CsvLogger(const std::string& path);
    void write(double time, const State& state,
               double requested_force, double applied_force, double external_force = 0.0, int stage = 0);
    bool good() const { return file_.good(); }

private:
    std::ofstream file_;
};
