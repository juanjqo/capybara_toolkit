#pragma once
#include <chrono>
#include <iostream>

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
    std::chrono::duration<double> elapsed_seconds_;
    //std::chrono::hh_mm_ss<std::chrono::duration<double>> elapsed_hhmmss_time_;
    double elapsed_time_;
    double _get_inverse_scale_factor(const SCALE& scale);
    std::string fmt_;
public:
    Cronos();
    void tic();
    double toc(const SCALE& scale = SCALE::SECONDS);
    double get_elapsed_time(const SCALE& scale = SCALE::SECONDS);
    void show_elapsed_time(const SCALE& scale = SCALE::SECONDS);
    //void show_elapsed_hhmmss_time();
};

}


