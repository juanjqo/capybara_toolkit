#include "capybara/conversions.hpp"

namespace Capybara {


Eigen::VectorXd Capybara::Conversions::std_vector_double_to_vectorxd(std::vector<double> &std_vector)
{
    return Eigen::Map<VectorXd>(std_vector.data(), std_vector.size());
}

VectorXd Conversions::std_vector_vectorxd_to_vectorxd(std::vector<VectorXd> &std_vectorxd)
{
    VectorXd q;
    for (auto &vec : std_vectorxd)
    {
        q = Capybara::Numpy::vstack(q, vec);
    }
    return q;
}

}
