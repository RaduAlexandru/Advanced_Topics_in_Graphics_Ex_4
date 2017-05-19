#ifndef ATCG1_LIB_CONVOLUTION_H
#define ATCG1_LIB_CONVOLUTION_H

#include <Eigen/Core>
double gauss_pdf(const double x, const double mean, const double sigma)
{
    return std::exp(- (x - mean) * (x - mean) / (2.0 * sigma * sigma));
}

template <typename Derived, typename Derived2 >
Derived conv2d(const Eigen::MatrixBase<Derived>& I, const Eigen::MatrixBase<Derived2> &kernel )
{
    int k_cols = kernel.cols();
    int k_rows = kernel.rows();

    assert(k_cols % 2 == 1); //odd kernelsize
    assert(k_rows % 2 == 1); //odd kernelsize

    Derived I_tmp = Derived::Zero(I.rows() + k_rows - 1, I.cols() + k_cols - 1);
    Derived O_tmp = Derived::Zero(I_tmp.rows(), I_tmp.cols());

    I_tmp.block(k_rows / 2, k_cols / 2, I.rows(), I.cols()) = I;


    typedef typename Derived::Scalar Scalar;
    typedef typename Derived2::Scalar Scalar2;

    Derived2 block ;

    for (int row = k_rows / 2; row < I_tmp.rows() - k_rows / 2; row++ )
    {
        for (int col = k_cols / 2; col < I_tmp.cols() - k_cols / 2; col++ )
        {
            Scalar b = (static_cast<Derived2>(I_tmp.block(row - k_rows / 2, col - k_cols / 2, k_rows, k_cols)).cwiseProduct(kernel)).sum();
            O_tmp.coeffRef(row, col) = b;
        }
    }

    Derived O = O_tmp.block(k_rows / 2, k_cols / 2, I.rows(), I.cols());

    return O;
}
#endif //ATCG1_LIB_CONVOLUTION_H
