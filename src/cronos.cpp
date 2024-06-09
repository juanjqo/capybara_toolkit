#include <capybara/cronos.hpp>

namespace Capybara {

double Cronos::_get_inverse_scale_factor(const SCALE &scale)
{
    switch(scale)
    {
    case SCALE::SECONDS:
        fmt_ = "s";
        return 1.0;
    case SCALE::MILLISECONDS:
        fmt_ = "ms";
        return 1e3;
    case SCALE::MICROSECONDS:
        fmt_ = "us";
        return 1e6 ;
    case SCALE::NANOSECONDS:
        fmt_ = "ns";
        return 1e9;
    }
}

Cronos::Cronos() {

}

void Cronos::tic()
{
    initial_time_ = std::chrono::steady_clock::now();
}

double Cronos::toc(const SCALE &scale)
{
    const auto end{std::chrono::steady_clock::now()};
    const std::chrono::duration<double> elapsed_seconds{end - initial_time_};
    elapsed_time_ = elapsed_seconds.count();
    return get_elapsed_time(scale);

}

double Cronos::get_elapsed_time(const SCALE &scale)
{
    return elapsed_time_*_get_inverse_scale_factor(scale);
}

void Cronos::show_elapsed_time(const SCALE &scale)
{
    std::cout<<std::format("Elapsed time: {} ({})", get_elapsed_time(scale), fmt_)<<std::endl;
}

}
