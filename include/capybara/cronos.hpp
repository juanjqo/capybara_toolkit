#pragma once
#include <chrono>

namespace Capybara {

class Cronos
{
public:
    enum class SCALE{
        SECONDS,
        MILLISECONDS,
        MICROSECONDS,
        NANOSECONDS
    };
private:
    std::chrono::time_point<std::chrono::steady_clock> initial_time_;
    double _get_inverse_scale_factor(const SCALE& scale);
public:
    Cronos();
    void tic();
    double toc(const SCALE& scale = SCALE::SECONDS);
};

}


