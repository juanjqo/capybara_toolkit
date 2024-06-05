#pragma once
#include <capybara/checkers.hpp>
#include <capybara/conversions.hpp>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

namespace Capybara {

class Numerical
{
public:
    static std::vector<MatrixXd> numerical_matrix_differentiation(const std::vector<Eigen::MatrixXd>& J,
                                                                  const double& T);

};

}
