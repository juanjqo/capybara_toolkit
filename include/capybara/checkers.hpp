#pragma once


#include <capybara/capytypes.hpp>
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


    static bool check_constraint_sizes(const MatrixXd& A,
                                       const VectorXd& b,
                                       const double& optimization_vector_size,
                                       const MODE& mode = Checkers::MODE::PANIC);

    template<typename T, typename U>
    static bool check_equal_sizes(const T &v1,
                                  const U &v2,
                                  const MODE& mode = Checkers::MODE::PANIC)
    {
        std::size_t s1 = static_cast<std::size_t>(v1.size());
        std::size_t s2 = static_cast<std::size_t>(v2.size());
        if (s1 != s2)
        {
            if (mode == Checkers::MODE::PANIC)
                throw std::runtime_error("Panic with Capybara::Checkers::check_equal_sizes(v1, v2). Both containers have diferrent sizes. ");
            return FAIL;
        }
        return SUCCESS;
    }

    template<typename T, typename U, typename V>
    static bool check_equal_sizes(const T &v1,
                                  const U &v2,
                                  const V &v3,
                                  const MODE& mode = Checkers::MODE::PANIC)
    {
        bool s1 = Checkers::check_equal_sizes(v1, v2, mode);
        bool s2 = Checkers::check_equal_sizes(v2, v3, mode);
        if (s1 != s2)
        {
            return FAIL;
        }
        return SUCCESS;
    }



};


}


