#pragma once
#include <capybara/checkers.hpp>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

namespace Capybara {

class Capynum
{
public:
    static VectorXd std_vector_double_to_vectorxd(std::vector<double> &std_vector);
    static VectorXd std_vector_vectorxd_to_vectorxd(std::vector<VectorXd>& std_vectorxd);
    static MatrixXd vstack(const MatrixXd& A, const MatrixXd& B);
    static MatrixXd hstack(const MatrixXd& A, const MatrixXd& B);
    static MatrixXd block_diag(const std::vector<MatrixXd>& A);
    static MatrixXd block_diag(const std::vector<double>& A);
    static MatrixXd resize(const MatrixXd& A, const int& rows, const int& cols);
    static VectorXd linspace(const double& start, const double& stop, const int& size);
    static MatrixXd linspace(const VectorXd& start, const VectorXd& stop, const int& size);

};
}


