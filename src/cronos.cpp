
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
    default: // This line is required in GCC compiler
        throw std::runtime_error("Wrong arguments.");
        break;
    }
}

/*
Cronos::Cronos() {

}*/

Cronos::Cronos(const double &frequency)
{
    time_per_iteration_in_milliseconds_ = (1/frequency)*1000.0;
    tic();
}

void Cronos::tic()
{
    initial_time_ = std::chrono::steady_clock::now();
}

double Cronos::toc(const SCALE &scale)
{
    const auto end{std::chrono::steady_clock::now()};
    //const std::chrono::duration<double> elapsed_seconds{end - initial_time_};
    elapsed_seconds_ = std::chrono::duration<double>{end - initial_time_};
    //elapsed_hhmmss_time_ = std::chrono::hh_mm_ss{elapsed_seconds_};
    elapsed_time_ = elapsed_seconds_.count();
    return get_elapsed_time(scale);

}

double Cronos::get_elapsed_time(const SCALE &scale)
{
    return elapsed_time_*_get_inverse_scale_factor(scale);
}

void Cronos::show_elapsed_time(const SCALE &scale)
{
    std::cout<<"Elapsed time: "<<get_elapsed_time(scale)<<"() "<<fmt_<<std::endl;
}

void Cronos::sleep()
{
    double elapsed_time = toc(SCALE::MILLISECONDS);
    double diff_time = time_per_iteration_in_milliseconds_-elapsed_time;
    diff_time = Capybara::Numpy::round(diff_time, 1);
    int error = diff_time;
    if (error<=0)
    {
       // std::cerr<<"Overrun!"<<std::endl;
    }else
        Capybara::millidelay(error);
    tic();
}

/*
void Cronos::show_elapsed_hhmmss_time()
{
    std::cout<<"Elapsed time: "<<elapsed_hhmmss_time_<<std::endl;
}
*/
// std::chrono::hh_mm_ss<std::chrono::seconds> tod{std::chrono::seconds(secs)};
}
