#include <iostream>
#include <capybara.hpp>
#include <iomanip>
#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

#include <dqrobotics/DQ.h>
#include <dqcapybara.hpp>
//#include <print> C++23
//#include <expected> C++23

using namespace Eigen;

int main()
{
    //const auto start{std::chrono::steady_clock::now()};
    auto clock = Capybara::Cronos();
    clock.tic();
    //std::chrono::time_point ts = start;

    MatrixXd A = MatrixXd::Zero(2,3);
    A << 1,2,3,4,5,6;
    VectorXd b = VectorXd::Zero(2);
    b << 7,9;

    auto cm = Capybara::ConstraintsManager(3);
    cm.add_inequality_constraint(A,b);
    auto [A_, b_] = cm.get_inequality_constraints();
    auto v = Capybara::Conversions::double2vector(180, 10);
    std::cout<<v<<std::endl;
    //std::cout<<Capybara::Conversions::deg2rad(v).transpose()<<std::endl;

    auto lim = Capybara::CVectorXd({1,2,3,4,5});

    //std::cout<<lim.transpose()<<std::endl;
    auto M = Capybara::CMatrixXd({
                                 {1,2,3,4},
                                 {4,5,6,7},
                                 {8,9,10,11}
                                });
    auto [H, f] = Capybara::Numpy::symmetric_and_linear_component(M, 0.001, 2, VectorXd::Zero(3));

    Capybara::rot(0.5, DQ_robotics::k_);
    //std::cout<<H<<std::endl;
    //std::cout<<f<<std::endl;

    //const auto end{std::chrono::steady_clock::now()};
    //const std::chrono::duration<double> elapsed_seconds{end - start};
    //double ti = elapsed_seconds.count();
    using np = Capybara::Numpy;
    double num = 0.0000345;
    std::setprecision(10);

    auto x = Capybara::rotz(90);
    std::cout<<x<<std::endl;

    //std::println("number: {}, rounded: {}",num, np::round(num, 6)); C++23


    std::cout<<std::format("number: {}, rounded: {}",num, np::round(num, 6))<<std::endl;
    std::cout<<clock.toc()<<std::endl;
    clock.show_elapsed_time(Capybara::Cronos::SCALE::MICROSECONDS);
    clock.show_elapsed_hhmmss_time();




}
