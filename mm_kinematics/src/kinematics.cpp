#include <ros/ros.h>
#include <Eigen/Eigen>

#include <mm_math_util/rotation.h>

#include "mm_kinematics/kinematics.h"


namespace mm {


// Calculate the Jacobian of the overall system.
void Kinematics::jacobian(const JointVector& q, JacobianMatrix& J) {
    ArmJacobianMatrix Ja;
    BaseJacobianMatrix Jb;
    jacobians(q, Ja, Jb);
    J << Jb, Ja;
}

void Kinematics::calc_w_T_base(const JointVector& q,
                               Eigen::Affine3d& w_T_base) {
    DH_TF(T1, M_PI_2, 0, 0,    M_PI_2);
    DH_TF(T2, M_PI_2, 0, q(0), M_PI_2);
    DH_TF(T3, M_PI_2, 0, q(1), M_PI_2);
    DH_TF(T4, q(2),   0, 0,    0);

    w_T_base = T1 * T2 * T3 * T4;
}

void Kinematics::calc_w_T_arm(const JointVector& q, Eigen::Affine3d& w_T_arm) {
    Eigen::Affine3d w_T_base;
    calc_w_T_base(q, w_T_base);

    DH_TF(T5, 0, px, pz, -M_PI_2);
    DH_TF(T6, 0, 0,  py,  M_PI_2);

    w_T_arm = w_T_base * T5 * T6;
}

void Kinematics::calc_w_T_ee(const JointVector& q, Eigen::Affine3d& w_T_ee) {
    Eigen::Affine3d w_T_arm;
    calc_w_T_arm(q, w_T_arm);

    DH_TF(T7,  q(3), 0,  d1,  M_PI_2);
    DH_TF(T8,  q(4), a2, 0,   0);
    DH_TF(T9,  q(5), a3, 0,   0);
    DH_TF(T10, q(6), 0,  d4,  M_PI_2);
    DH_TF(T11, q(7), 0,  d5, -M_PI_2);
    DH_TF(T12, q(8), 0,  d6,  0);

    w_T_ee = w_T_arm * T7 * T8 * T9 * T10 * T11 * T12;
}

void Kinematics::calc_w_T_tool(const JointVector& q,
                               Eigen::Affine3d& w_T_tool) {
    Eigen::Affine3d w_T_ee;
    calc_w_T_ee(q, w_T_ee);
    DH_TF(ee_T_tool, 0, 0, d7,  0);
    w_T_tool = w_T_ee * ee_T_tool;
}

double Kinematics::manipulability(const JointVector& q) {
    ArmJacobianMatrix Ja;
    BaseJacobianMatrix Jb;
    jacobians(q, Ja, Jb);
    double m2 = (Ja * Ja.transpose()).determinant();

    // Protect against small negative results introduced by numerical errors.
    if (m2 < 0) {
        m2 = 0;
    }

    double m = sqrt(m2);
    return m;
}


void Kinematics::manipulability_gradient_numeric(const JointVector& q,
                                                 JointVector& m_grad,
                                                 double h) {
    m_grad = JointVector::Zero();
    JointMatrix I = JointMatrix::Identity();

    // first two and last joint do not affect the MI
    for (int i = 2; i < NUM_JOINTS - 1; ++i) {
        JointVector Ii = I.col(i);
        double m1 = Kinematics::manipulability(q + h*Ii);
        double m2 = Kinematics::manipulability(q - h*Ii);
        m_grad(i) = (m1 - m2) / (2*h);
    }
}


void Kinematics::manipulability_gradient_analytic(const JointVector& q,
                                                  JointVector& m_grad) {
    ArmJacobianMatrix Ja;
    BaseJacobianMatrix Jb;
    jacobians(q, Ja, Jb);

    Matrix6d JJT = Ja * Ja.transpose();
    // Eigen::ColPivHouseholderQR<Matrix6d> JJT_dec(JJT);
    Eigen::LDLT<Matrix6d> JJT_dec(JJT);

    // xb, yb, and q6 are zero
    ArmJacobianMatrix dJa_dtb, dJa_dq1, dJa_dq2, dJa_dq3, dJa_dq4, dJa_dq5;
    calc_dJa_dq(q, dJa_dtb, dJa_dq1, dJa_dq2, dJa_dq3, dJa_dq4, dJa_dq5);

    JointVector traces;
    traces << 0, 0,
           JJT_dec.solve(Ja * dJa_dtb.transpose()).trace(),
           JJT_dec.solve(Ja * dJa_dq1.transpose()).trace(),
           JJT_dec.solve(Ja * dJa_dq2.transpose()).trace(),
           JJT_dec.solve(Ja * dJa_dq3.transpose()).trace(),
           JJT_dec.solve(Ja * dJa_dq4.transpose()).trace(),
           JJT_dec.solve(Ja * dJa_dq5.transpose()).trace(),
           0;

    double m = manipulability(q);
    m_grad = m * traces;
}

void Kinematics::dh_transform(double q, double a, double d, double alpha,
                              Eigen::Affine3d& T) {
    double sq = std::sin(q);
    double cq = std::cos(q);
    double salpha = std::sin(alpha);
    double calpha = std::cos(alpha);

    T.matrix() << cq, -sq*calpha,  sq*salpha, a*cq,
                  sq,  cq*calpha, -cq*salpha, a*sq,
                  0,   salpha,     calpha,    d,
                  0,   0,          0,         1;
}

void Kinematics::calc_base_input_mapping(const JointVector& q, JointMatrix& B) {
    // Rotation matrix from base to world frame.
    Eigen::Matrix2d R_wb = rotation2d(q(2));
    B = JointMatrix::Identity();
    B.topLeftCorner<2, 2>() = R_wb;
}

} // namespace mm

