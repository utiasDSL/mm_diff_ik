#include <iosfwd>
#include <cppad/cg.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <cppad/example/cppad_eigen.hpp>


const std::string LIBRARY_NAME = "./libmmad";
const std::string LIBRARY_NAME_EXT = LIBRARY_NAME + CppAD::cg::system::SystemInfo<>::DYNAMIC_LIB_EXTENSION;


template<typename T>
using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template<typename Scalar>
using AffineT = Eigen::Transform<Scalar, 3, Eigen::Affine>;

using CGD = CppAD::cg::CG<double>;
using ADCG = CppAD::AD<CGD>;

static const ADCG AD_PI_2 = ADCG(M_PI_2);
static const ADCG AD_0 = ADCG(0);
static const ADCG AD_1 = ADCG(1);

// number of joints
static const int N = 9;


// Kinematic parameters
// Offset from base to arm
static constexpr double px = 0.27;
static constexpr double py = 0.01;
static constexpr double pz = 0.653;

// Arm D-H parameters
static constexpr double d1 = 0.1273;
static constexpr double a2 = -0.612;
static constexpr double a3 = -0.5723;
static constexpr double d4 = 0.163941;
static constexpr double d5 = 0.1157;
static constexpr double d6 = 0.0922;
static constexpr double d7 = 0.290;


template<typename Scalar>
void dh_transform(Scalar q, Scalar a, Scalar d, Scalar alpha,
                  AffineT<Scalar>& T) {
    Scalar sq = CppAD::sin(q);
    Scalar cq = CppAD::cos(q);
    Scalar salpha = CppAD::sin(alpha);
    Scalar calpha = CppAD::cos(alpha);

    T.matrix() << cq,       -sq*calpha,  sq*salpha, a*cq,
                  sq,        cq*calpha, -cq*salpha, a*sq,
                  Scalar(0), salpha,     calpha,    d,
                  Scalar(0), Scalar(0),  Scalar(0), Scalar(1);
}


template<typename Scalar>
void calc_transforms(std::vector<AffineT<Scalar>>& T, const Vector<Scalar>& q) {
    T.resize(13);

    dh_transform<Scalar>(Scalar(M_PI_2), Scalar(0), Scalar(0), Scalar(M_PI_2), T[0]);
    dh_transform<Scalar>(Scalar(M_PI_2), Scalar(0), q(0),      Scalar(M_PI_2), T[1]);
    dh_transform<Scalar>(Scalar(M_PI_2), Scalar(0), q(1),      Scalar(M_PI_2), T[2]);
    dh_transform<Scalar>(q(2),           Scalar(0), Scalar(0), Scalar(0),      T[3]);

    // Static base-arm transform
    dh_transform<Scalar>(Scalar(0), Scalar(px), Scalar(pz), -Scalar(M_PI_2), T[4]);
    dh_transform<Scalar>(Scalar(0), Scalar(0),  Scalar(py),  Scalar(M_PI_2), T[5]);

    // Arm
    dh_transform<Scalar>(q(3), Scalar(0),  Scalar(d1),  Scalar(M_PI_2), T[6]);
    dh_transform<Scalar>(q(4), Scalar(a2), Scalar(0),   Scalar(0),      T[7]);
    dh_transform<Scalar>(q(5), Scalar(a3), Scalar(0),   Scalar(0),      T[8]);
    dh_transform<Scalar>(q(6), Scalar(0),  Scalar(d4),  Scalar(M_PI_2), T[9]);
    dh_transform<Scalar>(q(7), Scalar(0),  Scalar(d5), -Scalar(M_PI_2), T[10]);
    dh_transform<Scalar>(q(8), Scalar(0),  Scalar(d6),  Scalar(0),      T[11]);

    // Tool
    dh_transform<Scalar>(Scalar(0), Scalar(0), Scalar(d7),  Scalar(0), T[12]);
}


template<typename Scalar>
void calc_transforms_world(std::vector<AffineT<Scalar>>& transforms, const Vector<Scalar>& q) {
    calc_transforms(transforms, q);
    for (int i = 1; i < transforms.size(); ++i) {
        transforms[i] = transforms[i-1] * transforms[i];
    }
}


void angular_jacobian(Eigen::Matrix<double, 3, N>& Jw, const Vector<double>& q) {
    std::vector<AffineT<double>> transforms;
    calc_transforms_world(transforms, q);

    Eigen::Vector3d k;
    k << 0, 0, 1;

    Jw.col(0) = 0 * transforms[1].rotation() * k;
    Jw.col(1) = 0 * transforms[2].rotation() * k;
    Jw.col(2) = transforms[3].rotation() * k;

    Jw.col(3) = transforms[6].rotation()  * k;
    Jw.col(4) = transforms[7].rotation()  * k;
    Jw.col(5) = transforms[8].rotation()  * k;
    Jw.col(6) = transforms[9].rotation() * k;
    Jw.col(7) = transforms[10].rotation() * k;
    Jw.col(8) = transforms[11].rotation() * k;
}


void gen_c_code(CppAD::ADFun<CGD>& f, std::ostringstream& code) {
    CppAD::cg::CodeHandler<double> handler;

    CppAD::vector<CGD> indVars(N);
    handler.makeVariables(indVars);

    CppAD::vector<CGD> jac = f.SparseJacobian(indVars);

    CppAD::cg::LanguageC<double> langC("double");
    // langC.setParameterPrecision(6);
    CppAD::cg::LangCDefaultVariableNameGenerator<double> nameGen;

    handler.generateCode(code, langC, jac, nameGen);
}


void gen_so_lib(CppAD::ADFun<CGD>& f, const std::string& name) {
    // generates source code
    CppAD::cg::ModelCSourceGen<double> cgen(f, name);
    cgen.setCreateJacobian(true);
    CppAD::cg::ModelLibraryCSourceGen<double> libcgen(cgen);

    // compile source code
    CppAD::cg::DynamicModelLibraryProcessor<double> p(libcgen, LIBRARY_NAME);
    CppAD::cg::GccCompiler<double> compiler;
    std::unique_ptr<CppAD::cg::DynamicLib<double>> dynamicLib = p.createDynamicLibrary(compiler);
}


int main(void) {
    /*** Model ***/

    // Independent variable is the joint positions q
    Vector<ADCG> q = Vector<ADCG>::Ones(N);
    CppAD::Independent(q);

    // Test that dh_transform works for regular doubles.
    // AffineT<double> T_test;
    // dh_transform<double>(M_PI_2, 0, 0, M_PI_2, T_test);
    // std::cout << T_test.matrix() << std::endl;

    std::vector<AffineT<ADCG>> transforms;
    calc_transforms_world(transforms, q);

    AffineT<ADCG> T_we = transforms.back();
    Vector<ADCG> y = T_we.translation();
    CppAD::ADFun<CGD> f(q, y);

    /*** C code gen ***/

    // std::ostringstream code;
    // gen_c_code(f, code);
    // std::cout << code.str();

    /*** Dynamic lib gen ***/

    const std::string model_name = "mm";

    gen_so_lib(f, model_name);

    // Load the dynamic library.
    CppAD::cg::LinuxDynamicLib<double> dynamicLib(LIBRARY_NAME_EXT);
    std::unique_ptr<CppAD::cg::GenericModel<double>> model = dynamicLib.model(model_name);

    Eigen::Matrix<double, N, 1> q_test = Eigen::Matrix<double, N, 1>::Random();
    // std::vector<double> q_test(N, 0.0);
    std::vector<double> Jv = model->Jacobian(std::vector<double>(q_test.data(), q_test.data() + q_test.size()));

    // Map back to an Eigen matrix.
    Eigen::Map<Eigen::Matrix<double, 3, N, Eigen::RowMajor>> J(Jv.data());
    std::cout << J << std::endl;

    Eigen::Matrix<double, 3, N> Jw;
    angular_jacobian(Jw, Eigen::Matrix<double, N, 1>(q_test));
    std::cout << Jw << std::endl;
}
