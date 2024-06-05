#include <capybara/numerical.hpp>

class Numerical
{

std::vector<Eigen::MatrixXd> numerical_matrix_differentiation(const std::vector<MatrixXd> &J,
                                                              const double &T)
{
    int s = J.size();
    std::vector<MatrixXd> J_dot(s, MatrixXd::Zero(J[0].rows(), J[0].cols()));

    for(int i=4; i<s-4;i++) //for(int i=2; i<s-2;i++)
    {
        //J_dot[i] = ((J[i-2]-8*J[i-1]+8*J[i+1]-J[i+2])/(12*T));  // four-point central finite difference

        //Generation of finite difference formulas on arbitrarily spaced grids
        //Bengt Fornberg, 1988
        //10.1090/S0025-5718-1988-0935077-0
        J_dot[i] = (3*J[i-4]- 32*J[i-3]+168*J[i-2]-672*J[i-1]+    // eight-point central finite difference
                    672*J[i+1]-168*J[i+2]+ 32*J[i+3]-  3*J[i+4])/(840*T);
    }
    return J_dot;

};

};
