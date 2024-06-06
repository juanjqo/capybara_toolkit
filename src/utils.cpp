#include <capybara/utils.hpp>

#include <iostream>

VectorXd Capybara::CVectorXd(const std::vector<double> &vec)
{
    std::vector<double> myvec = vec;
    return Capybara::Conversions::std_vector_double_to_vectorxd(myvec);
}

MatrixXd Capybara::CMatrixXd(const std::vector<std::vector<double> > &mat)
{
    int rows = mat.size();
    std::vector<int> sizes(rows, 0);
    for (auto i=0;i<rows;i++)
        sizes.at(i) = mat.at(i).size();

    if (!Capybara::Checkers::check_for_equal_elements(sizes, Capybara::Checkers::MODE::DO_NOT_PANIC))
        throw std::runtime_error("Panic with Capybara::MatrixXd(). Wrong number of elements. "
                                 "All vector must have the same dimension.  ");

    int cols = sizes.at(0);
    MatrixXd output = MatrixXd(rows,cols);
    auto mymat = mat;
    for (auto i=0;i<rows;i++)
        output.row(i) = Capybara::Conversions::std_vector_double_to_vectorxd(mymat.at(i));

    return output;
}
