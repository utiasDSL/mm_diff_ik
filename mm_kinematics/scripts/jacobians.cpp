
#include <ros/ros.h>
#include <Eigen/Eigen>

#include "mm_kinematics/kinematics.h"

namespace mm {

void Kinematics::jacobians(const JointVector& q, ArmJacobianMatrix& Ja, BaseJacobianMatrix& Jb) {
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

// Base Jacobian
Jb(0,0) = 1;
Jb(0,1) = 0;
Jb(0,2) = a2*(-sq1*ctb - stb*cq1)*cq2 - a3*(-sq1*ctb - stb*cq1)*sq2*sq3 + a3*(-sq1*ctb - stb*cq1)*cq2*cq3 + d4*(-sq1*stb + cq1*ctb) + d5*(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4) + d6*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5) + d7*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5) - px*stb - py*ctb;
Jb(1,0) = 0;
Jb(1,1) = 1;
Jb(1,2) = a2*(-sq1*stb + cq1*ctb)*cq2 - a3*(-sq1*stb + cq1*ctb)*sq2*sq3 + a3*(-sq1*stb + cq1*ctb)*cq2*cq3 + d4*(sq1*ctb + stb*cq1) + d5*(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*cq4) + d6*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5) + d7*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5) + px*ctb - py*stb;
Jb(2,0) = 0;
Jb(2,1) = 0;
Jb(2,2) = 0;
Jb(3,0) = 0;
Jb(3,1) = 0;
Jb(3,2) = 0;
Jb(4,0) = 0;
Jb(4,1) = 0;
Jb(4,2) = 0;
Jb(5,0) = 0;
Jb(5,1) = 0;
Jb(5,2) = 1;

// Arm Jacobian
Ja(0,0) = a2*(-sq1*ctb - stb*cq1)*cq2 - a3*(-sq1*ctb - stb*cq1)*sq2*sq3 + a3*(-sq1*ctb - stb*cq1)*cq2*cq3 + d4*(-sq1*stb + cq1*ctb) + d5*(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4) + d6*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5) + d7*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5);
Ja(0,1) = -a2*(-sq1*stb + cq1*ctb)*sq2 - a3*(-sq1*stb + cq1*ctb)*sq2*cq3 - a3*(-sq1*stb + cq1*ctb)*sq3*cq2 + d5*((-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4 + ((sq1*stb - cq1*ctb)*sq2*sq3 - (sq1*stb - cq1*ctb)*cq2*cq3)*cq4) + d6*(-(-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5 + d7*(-(-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5;
Ja(0,2) = -a3*(-sq1*stb + cq1*ctb)*sq2*cq3 - a3*(-sq1*stb + cq1*ctb)*sq3*cq2 + d5*((-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*sq4 + ((sq1*stb - cq1*ctb)*sq2*sq3 - (sq1*stb - cq1*ctb)*cq2*cq3)*cq4) + d6*(-(-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5 + d7*(-(-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5;
Ja(0,3) = d5*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - ((-sq1*stb + cq1*ctb)*sq2*cq3 + (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4) + d6*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq5 + d7*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq5;
Ja(0,4) = d6*((-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5) + d7*((-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5);
Ja(0,5) = 0;
Ja(1,0) = a2*(-sq1*stb + cq1*ctb)*cq2 - a3*(-sq1*stb + cq1*ctb)*sq2*sq3 + a3*(-sq1*stb + cq1*ctb)*cq2*cq3 + d4*(sq1*ctb + stb*cq1) + d5*(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*cq4) + d6*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5) + d7*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5);
Ja(1,1) = -a2*(sq1*ctb + stb*cq1)*sq2 - a3*(sq1*ctb + stb*cq1)*sq2*cq3 - a3*(sq1*ctb + stb*cq1)*sq3*cq2 + d5*(((-sq1*ctb - stb*cq1)*sq2*sq3 - (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*sq4) + d6*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*sq5 + d7*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*sq5;
Ja(1,2) = -a3*(sq1*ctb + stb*cq1)*sq2*cq3 - a3*(sq1*ctb + stb*cq1)*sq3*cq2 + d5*(((-sq1*ctb - stb*cq1)*sq2*sq3 - (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4) + d6*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5 + d7*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5;
Ja(1,3) = d5*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4) + d6*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5 + d7*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5;
Ja(1,4) = d6*((-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (sq1*stb - cq1*ctb)*sq5) + d7*((-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (sq1*stb - cq1*ctb)*sq5);
Ja(1,5) = 0;
Ja(2,0) = 0;
Ja(2,1) = a2*cq2 - a3*sq2*sq3 + a3*cq2*cq3 + d5*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5 - d7*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5;
Ja(2,2) = -a3*sq2*sq3 + a3*cq2*cq3 + d5*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5 - d7*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5;
Ja(2,3) = d5*(-(sq2*sq3 - cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 - (sq2*cq3 + sq3*cq2)*sq4)*sq5 - d7*((-sq2*sq3 + cq2*cq3)*cq4 - (sq2*cq3 + sq3*cq2)*sq4)*sq5;
Ja(2,4) = -d6*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*cq5 - d7*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*cq5;
Ja(2,5) = 0;
Ja(3,0) = 0;
Ja(3,1) = sq1*ctb + stb*cq1;
Ja(3,2) = sq1*ctb + stb*cq1;
Ja(3,3) = sq1*ctb + stb*cq1;
Ja(3,4) = (-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4;
Ja(3,5) = -((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5;
Ja(4,0) = 0;
Ja(4,1) = sq1*stb - cq1*ctb;
Ja(4,2) = sq1*stb - cq1*ctb;
Ja(4,3) = sq1*stb - cq1*ctb;
Ja(4,4) = (-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4;
Ja(4,5) = -((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5;
Ja(5,0) = 1;
Ja(5,1) = 0;
Ja(5,2) = 0;
Ja(5,3) = 0;
Ja(5,4) = -(-sq2*sq3 + cq2*cq3)*cq4 + (sq2*cq3 + sq3*cq2)*sq4;
Ja(5,5) = -((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5;

}

} // namespace mm
