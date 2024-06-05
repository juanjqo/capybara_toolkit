#include <iostream>
#include <capybara.hpp>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

int main()
{
    MatrixXd A = MatrixXd::Zero(2,3);
    A << 1,2,3,4,5,6;
    VectorXd b = VectorXd::Zero(2);
    b << 7,9;

    auto cm = Capybara::ConstraintsManager(3);
    cm.add_inequality_constraint(A,b);
    auto [A_, b_] = cm.get_inequality_constraints();
    std::cout<<A_<<std::endl;
    std::cout<<b_<<std::endl;
}
