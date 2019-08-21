#pragma once

#include <Eigen/Eigen>


namespace mm {

// Approximate gradient using central differences.
// f: Function mapping N-dim vector input to scalar output
// h: Step size
// x: Input point
// grad: The gradient, populated by this function.
template<int N>
void approx_gradient(double (*f)(const Eigen::Matrix<double, N, 1>&),
                     double h, const Eigen::Matrix<double, N, 1>& x,
                     Eigen::Matrix<double, N, 1>& grad) {
    typedef Eigen::Matrix<double, N, N> MatrixNd;
    typedef Eigen::Matrix<double, N, 1> VectorNd;

    MatrixNd I = MatrixNd::Identity();

    for (int i = 0; i < N; ++i) {
        VectorNd Ii = I.col(i);
        grad(i) = (f(x + h*Ii) - f(x - h*Ii)) / (2*h);
    }
}


// Approximate Hessian using central differences.
// f: Function mapping N-dim vector input to scalar output
// h: Step size
// x: Input point
// H: The Hessian, populated by this function.
template<int N>
void approx_hessian(double (*f)(const Eigen::Matrix<double, N, 1>&),
                    double h, const Eigen::Matrix<double, N, 1>& x,
                    Eigen::Matrix<double, N, N>& H) {
    typedef Eigen::Matrix<double, N, N> MatrixNd;
    typedef Eigen::Matrix<double, N, 1> VectorNd;

    MatrixNd I = MatrixNd::Identity();

    // Note we exploit the fact that the Hessian is symmetric to avoid
    // computing every single entry.
    for (int i = 0; i < N; ++i) {
        VectorNd Ii = I.col(i);
        for (int j = i; j < N; ++j) {
            VectorNd Ij = I.col(j);
            H(i,j) = (f(x + h*Ii + h*Ij) - f(x + h*Ii - h*Ij)
                    - f(x - h*Ii + h*Ij) + f(x - h*Ii - h*Ij)) / (4*h*h);
            H(j,i) = H(i,j);
        }
    }
}

} // namespace mm
