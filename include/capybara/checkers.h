#pragma once


#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


using namespace Eigen;

namespace Capybara {

class Checkers
{
public:
    enum class MODE{PANIC, DO_NOT_PANIC};
    static bool check_column_matrix_sizes(const MatrixXd& A,
                                          const MatrixXd& B,
                                          const Checkers::MODE& mode = Checkers::MODE::PANIC);

    static bool check_row_matrix_sizes(const MatrixXd& A,
                                       const MatrixXd& B,
                                       const MODE& mode = Checkers::MODE::PANIC);

};


}


