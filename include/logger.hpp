#pragma once

#include "types.hpp"

#include <fstream>
#include <string>

class CsvLogger {
public:
    explicit CsvLogger(const std::string& path);
    void write(double time, const State& state,
               double requested_force, double applied_force);
    bool good() const { return file_.good(); }

private:
    std::ofstream file_;
};
