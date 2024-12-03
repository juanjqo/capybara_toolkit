#pragma once
#include <chrono>
#include <iostream>
#include "capybara/utils.hpp"

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
    double time_per_iteration_in_milliseconds_;
    int frequency_;
    double elapsed_time_;
    double _get_inverse_scale_factor(const SCALE& scale);
    std::string fmt_;
public:
    //Cronos();
    Cronos(const double& frequency = 100);
    void tic();
    double toc(const SCALE& scale = SCALE::SECONDS);
    double get_elapsed_time(const SCALE& scale = SCALE::SECONDS);
    void show_elapsed_time(const SCALE& scale = SCALE::SECONDS);

    void sleep();
    //void show_elapsed_hhmmss_time();
};

}


