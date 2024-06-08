#include <capybara/cronos.hpp>

namespace Capybara {

double Cronos::_get_inverse_scale_factor(const SCALE &scale)
{
    switch(scale)
    {
    case SCALE::SECONDS:
        return 1.0;
    case SCALE::MILLISECONDS:
        return 1e3;
    case SCALE::MICROSECONDS:
        return 1e6 ;
    case SCALE::NANOSECONDS:
        return 1e9;
    }
}

Cronos::Cronos() {

}

void Cronos::tic()
{
    initial_time_ = std::chrono::steady_clock::now();
}

double Cronos::toc(const SCALE& scale)
{
    const auto end{std::chrono::steady_clock::now()};
    const std::chrono::duration<double> elapsed_seconds{end - initial_time_};
    return elapsed_seconds.count()*_get_inverse_scale_factor(scale);
}

}
