#include <iostream>
#include "capybara.h"

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

int main()
{
    auto A = MatrixXd(3,3);
    auto B = MatrixXd(3,3);

    auto flag = Capybara::Checkers::check_column_matrix_sizes(A,B,Capybara::Checkers::MODE::PANIC);
    std::cout<<"flag: "<<flag<<std::endl;

    auto flag2 = Capybara::Checkers::check_row_matrix_sizes(A,B,Capybara::Checkers::MODE::PANIC);
    std::cout<<"flag2: "<<flag2<<std::endl;

    return 0;
}
