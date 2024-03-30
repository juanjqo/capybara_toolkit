#include <iostream>
#include "capybara.hpp"

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

int main()
{
    auto A = MatrixXd::Identity(3,3);
    auto B = MatrixXd(3,3);

    auto flag = Capybara::Checkers::check_column_matrix_sizes(A,B,Capybara::Checkers::MODE::PANIC);
    std::cout<<"flag: "<<flag<<std::endl;

    auto flag2 = Capybara::Checkers::check_row_matrix_sizes(A,B,Capybara::Checkers::MODE::PANIC);
    std::cout<<"flag2: "<<flag2<<std::endl;

    std::vector<MatrixXd> VM = {A,B};
    auto C = Capybara::Capynum::block_diag({A,B});
    std::cout<<C<<std::endl;

    std::cout<<Capybara::Capynum::linspace(0,10,5).transpose()<<std::endl;

    VectorXd v1 = (VectorXd(4)<<1,2,3,4).finished();
    VectorXd v2 = (VectorXd(4)<<10,20,30,40).finished();
    std::vector<VectorXd> VV = {v1,v2};

    std::cout<<Capybara::Capynum::linspace(v1, v2, 5)<<std::endl;
    std::cout<<Capybara::Capynum::std_vector_vectorxd_to_vectorxd(VV);

    Capybara::Checkers::check_equal_sizes(VV,VM);

    return 0;
}