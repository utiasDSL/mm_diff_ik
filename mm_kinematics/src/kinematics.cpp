#include <ros/ros.h>
#include <Eigen/Eigen>

#include "mm_kinematics/kinematics.h"


namespace mm {


// Calculate the Jacobian of the overall system.
void Kinematics::jacobian(const JointVector& q, JacobianMatrix& J) {
    ArmJacobianMatrix Ja;
    BaseJacobianMatrix Jb;
    jacobians(q, Ja, Jb);
    J << Jb, Ja;
}

// Calculate base and arm Jacobians.
void Kinematics::jacobians(const JointVector& q, ArmJacobianMatrix& Ja,
                           BaseJacobianMatrix& Jb) {
    // Base joints
    double xb = q(0);
    double yb = q(1);
    double stb = std::sin(q(2));
    double ctb = std::cos(q(2));

    // Arm joints
    double sq1 = std::sin(q(3));
    double cq1 = std::cos(q(3));
    double sq2 = std::sin(q(4));
    double cq2 = std::cos(q(4));
    double sq3 = std::sin(q(5));
    double cq3 = std::cos(q(5));
    double sq4 = std::sin(q(6));
    double cq4 = std::cos(q(6));
    double sq5 = std::sin(q(7));
    double cq5 = std::cos(q(7));
    double sq6 = std::sin(q(8));
    double cq6 = std::cos(q(8));

    // Arm Jacobian Ja = dpe/dqa
    double Ja00 = a2*(-sq1*ctb - stb*cq1)*cq2
        - a3*(-sq1*ctb - stb*cq1)*sq2*sq3
        + a3*(-sq1*ctb - stb*cq1)*cq2*cq3
        + d4*(-sq1*stb + cq1*ctb)
        + d5*(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)
        + d6*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5);
    double Ja01 = -a2*(-sq1*stb + cq1*ctb)*sq2
        - a3*(-sq1*stb + cq1*ctb)*sq2*cq3
        - a3*(-sq1*stb + cq1*ctb)*sq3*cq2
        + d5*((-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4 + ((sq1*stb - cq1*ctb)*sq2*sq3 - (sq1*stb - cq1*ctb)*cq2*cq3)*cq4)
        + d6*(-(-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5;
    double Ja02 = -a3*(-sq1*stb + cq1*ctb)*sq2*cq3
        - a3*(-sq1*stb + cq1*ctb)*sq3*cq2
        + d5*((-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*sq4 + ((sq1*stb - cq1*ctb)*sq2*sq3 - (sq1*stb - cq1*ctb)*cq2*cq3)*cq4)
        + d6*(-(-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5;
    double Ja03 = d5*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - ((-sq1*stb + cq1*ctb)*sq2*cq3 + (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)
        + d6*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq5;
    double Ja04 = d6*((-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5);
    double Ja05 = 0;

    double Ja10 = a2*(-sq1*stb + cq1*ctb)*cq2
        - a3*(-sq1*stb + cq1*ctb)*sq2*sq3
        + a3*(-sq1*stb + cq1*ctb)*cq2*cq3
        + d4*(sq1*ctb + stb*cq1)
        + d5*(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*cq4)
        + d6*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5);
    double Ja11 = -a2*(sq1*ctb + stb*cq1)*sq2
        - a3*(sq1*ctb + stb*cq1)*sq2*cq3
        - a3*(sq1*ctb + stb*cq1)*sq3*cq2
        + d5*(((-sq1*ctb - stb*cq1)*sq2*sq3 - (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*sq4)
        + d6*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*sq5;
    double Ja12 = -a3*(sq1*ctb + stb*cq1)*sq2*cq3
        - a3*(sq1*ctb + stb*cq1)*sq3*cq2
        + d5*(((-sq1*ctb - stb*cq1)*sq2*sq3 - (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)
        + d6*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5;
    double Ja13 = d5*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)
        + d6*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5;
    double Ja14 = d6*((-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (sq1*stb - cq1*ctb)*sq5);
    double Ja15 = 0;

    double Ja20 = 0;
    double Ja21 = a2*cq2 - a3*sq2*sq3 + a3*cq2*cq3
        + d5*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)
        - d6*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5;
    double Ja22 = -a3*sq2*sq3 + a3*cq2*cq3
        + d5*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)
        - d6*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5;
    double Ja23 = d5*(-(sq2*sq3 - cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 - (sq2*cq3 + sq3*cq2)*sq4)*sq5;
    double Ja24 = -d6*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*cq5;
    double Ja25 = 0;

    double Ja30 = sq1*ctb + stb*cq1;
    double Ja31 = sq1*ctb + stb*cq1;
    double Ja32 = sq1*ctb + stb*cq1;
    double Ja33 = (-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4
        - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4;
    double Ja34 = -((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5
        + (sq1*ctb + stb*cq1)*cq5;
    double Ja35 = -((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5
        + (sq1*ctb + stb*cq1)*cq5;

    double Ja40 = sq1*stb - cq1*ctb;
    double Ja41 = sq1*stb - cq1*ctb;
    double Ja42 = sq1*stb - cq1*ctb;
    double Ja43 = (-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4;
    double Ja44 = -((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5;
    double Ja45 = -((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5;

    double Ja50 = 0;
    double Ja51 = 0;
    double Ja52 = 0;
    double Ja53 = -(-sq2*sq3 + cq2*cq3)*cq4 + (sq2*cq3 + sq3*cq2)*sq4;
    double Ja54 = -((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5;
    double Ja55 = -((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5;

    Ja << Ja00, Ja01, Ja02, Ja03, Ja04, Ja05,
          Ja10, Ja11, Ja12, Ja13, Ja14, Ja15,
          Ja20, Ja21, Ja22, Ja23, Ja24, Ja25,
          Ja30, Ja31, Ja32, Ja33, Ja34, Ja35,
          Ja40, Ja41, Ja42, Ja43, Ja44, Ja45,
          Ja50, Ja51, Ja52, Ja53, Ja54, Ja55;

    // Base Jacobian Jb = dpe/dqb
    double Jb02 = a2*(-sq1*ctb - stb*cq1)*cq2
        - a3*(-sq1*ctb - stb*cq1)*sq2*sq3
        + a3*(-sq1*ctb - stb*cq1)*cq2*cq3
        + d4*(-sq1*stb + cq1*ctb)
        + d5*(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)
        + d6*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5)
        - px*stb - py*ctb;

    double Jb12 = a2*(-sq1*stb + cq1*ctb)*cq2
        - a3*(-sq1*stb + cq1*ctb)*sq2*sq3
        + a3*(-sq1*stb + cq1*ctb)*cq2*cq3
        + d4*(sq1*ctb + stb*cq1)
        + d5*(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*cq4)
        + d6*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5)
        + px*ctb - py*stb;

    Jb << 1, 0, Jb02,
          0, 1, Jb12,
          0, 0, 0,
          0, 0, 0,
          0, 0, 0,
          0, 0, 1;
}



void Kinematics::calc_w_T_b(const JointVector& q, Eigen::Affine3d& w_T_b) {
    DH_TF(T1, M_PI_2, 0, 0,    M_PI_2);
    DH_TF(T2, M_PI_2, 0, q(0), M_PI_2);
    DH_TF(T3, M_PI_2, 0, q(1), M_PI_2);
    DH_TF(T4, q(2),   0, 0,    0);

    w_T_b = T1 * T2 * T3 * T4;
}

void Kinematics::calc_w_T_a(const JointVector& q, Eigen::Affine3d& w_T_a) {
    Eigen::Affine3d w_T_b;
    calc_w_T_b(q, w_T_b);

    DH_TF(T5, 0, px, pz, -M_PI_2);
    DH_TF(T6, 0, 0,  py,  M_PI_2);

    w_T_a = w_T_b * T5 * T6;
}

void Kinematics::calc_w_T_e(const JointVector& q, Eigen::Affine3d& w_T_e) {
    Eigen::Affine3d w_T_a;
    calc_w_T_a(q, w_T_a);

    DH_TF(T7,  q(3), 0,  d1,  M_PI_2);
    DH_TF(T8,  q(4), a2, 0,   0);
    DH_TF(T9,  q(5), a3, 0,   0);
    DH_TF(T10, q(6), 0,  d4,  M_PI_2);
    DH_TF(T11, q(7), 0,  d5, -M_PI_2);
    DH_TF(T12, q(8), 0,  d6,  0);

    w_T_e = w_T_a * T7 * T8 * T9 * T10 * T11 * T12;
}

// TODO Legacy method: remove in favour of above
void Kinematics::forward(const JointVector& q, Eigen::Affine3d& w_T_e) {
    calc_w_T_e(q, w_T_e);
}

void Kinematics::forward_vel(const JointVector& q, const JointVector& dq,
                             Vector6d& v) {
    JacobianMatrix J;
    Kinematics::jacobian(q, J);
    v = J * dq;
}

double Kinematics::manipulability(const JointVector& q) {
    // JacobianMatrix J;
    // jacobian(q, J);
    // double m = sqrt((J * J.transpose()).determinant());
    // return m;
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

} // namespace mm

