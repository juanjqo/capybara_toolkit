#pragma once
#include <capybara/capytypes.hpp>
#include <capybara/numpy.hpp>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

namespace Capybara {

class Conversions
{
public:
    static VectorXd std_vector_double_to_vectorxd(std::vector<double> &std_vector);
    static VectorXd std_vector_vectorxd_to_vectorxd(std::vector<VectorXd>& std_vectorxd);
};
}

