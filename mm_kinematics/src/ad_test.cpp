# include <iostream>        // standard input/output
# include <vector>          // standard vector
# include <cppad/cppad.hpp> // the CppAD package
# include <Eigen/Eigen>
# include <cppad/example/cppad_eigen.hpp>


// CppAD only works with dynamically-sized vectors for some reason
template<typename T>
using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Custom scalar type for AD
using ADdouble = CppAD::AD<double>;


int main(void) {
    // Independent variable (take derivative w.r.t. this)
    Vector<ADdouble> x = Vector<ADdouble>::Ones(3);
    CppAD::Independent(x);

    // Perform operations on independent variable. Note all scalars must be of
    // type ADdouble
    Eigen::Matrix<double, 3, 3> A = Eigen::Matrix<double, 3, 3>::Random();
    Vector<ADdouble> y = A.cast<ADdouble>() * x;
    CppAD::ADFun<double> f(x, y);

    // Evaluate Jacobian
    Vector<double> x_test = Vector<double>::Ones(3);
    Vector<double> J = f.Jacobian(x_test);

    // Remap back to a matrix from a vector
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> Jac(J.data());
    std::cout << Jac << std::endl;

    std::cout << A << std::endl;
}
