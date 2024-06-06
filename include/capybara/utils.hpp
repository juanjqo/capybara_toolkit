#pragma once
#include <capybara/conversions.hpp>
#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


using namespace Eigen;

namespace Capybara {

VectorXd CVectorXd(const std::vector<double>& vec);
MatrixXd CMatrixXd(const std::vector<std::vector<double>>& mat);

}

