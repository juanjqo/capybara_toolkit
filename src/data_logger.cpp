#include <capybara/data_logger.hpp>



Capybara::DataLogger::DataLogger(const TYPE &type):type_{type}
{

}

void Capybara::DataLogger::add_data(const VectorXd &data)
{
    if (type_ != TYPE::VECTORXD)
        throw std::runtime_error("Panic with Capybara::DataLogger::add_data: Wrong type of data.");
    if (first_call_)
    {
        vector_size_ =  data.size();
        matrix_data_ = data;
        first_call_ = false;
    }
    else{
        matrix_data_ = Capybara::Numpy::resize(matrix_data_, matrix_data_.rows(), matrix_data_.cols()+1);
        i_ = i_ + 1;
        matrix_data_.block(0,i_, vector_size_, 1) = data;
    }



}

void Capybara::DataLogger::save_data(const std::string &filename)
{

    data_logger_.open(filename + ".csv");
    int m = matrix_data_.rows();
    int n = matrix_data_.cols();
    for (int j=0; j<n;j++)
    {
        for (int i=0;i<m;i++)
        {
            if (i<m-1)
                data_logger_ << matrix_data_(i,j)<<",";
            else
                data_logger_ << matrix_data_(i,j)<<","<<'\n';
        }
    }
    data_logger_.close();
}

void Capybara::DataLogger::show_data()
{
    std::cout<<matrix_data_<<std::endl;
}
