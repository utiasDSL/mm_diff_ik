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

    // Arm Jacobian Ja
    // Ja(0,0) = a2*(-sq1*ctb - stb*cq1)*cq2 - a3*(-sq1*ctb - stb*cq1)*sq2*sq3 + a3*(-sq1*ctb - stb*cq1)*cq2*cq3 + d4*(-sq1*stb + cq1*ctb) + d5*(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4) + d6*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5) + d7*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5);
    // Ja(0,1) = -a2*(-sq1*stb + cq1*ctb)*sq2 - a3*(-sq1*stb + cq1*ctb)*sq2*cq3 - a3*(-sq1*stb + cq1*ctb)*sq3*cq2 + d5*((-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4 + ((sq1*stb - cq1*ctb)*sq2*sq3 - (sq1*stb - cq1*ctb)*cq2*cq3)*cq4) + d6*(-(-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5 + d7*(-(-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5;
    // Ja(0,2) = -a3*(-sq1*stb + cq1*ctb)*sq2*cq3 - a3*(-sq1*stb + cq1*ctb)*sq3*cq2 + d5*((-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*sq4 + ((sq1*stb - cq1*ctb)*sq2*sq3 - (sq1*stb - cq1*ctb)*cq2*cq3)*cq4) + d6*(-(-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5 + d7*(-(-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5;
    // Ja(0,3) = d5*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - ((-sq1*stb + cq1*ctb)*sq2*cq3 + (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4) + d6*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq5 + d7*((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq5;
    // Ja(0,4) = d6*((-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5) + d7*((-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5);
    // Ja(0,5) = 0;
    //
    // Ja(1,0) = a2*(-sq1*stb + cq1*ctb)*cq2 - a3*(-sq1*stb + cq1*ctb)*sq2*sq3 + a3*(-sq1*stb + cq1*ctb)*cq2*cq3 + d4*(sq1*ctb + stb*cq1) + d5*(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*cq4) + d6*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5) + d7*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5);
    // Ja(1,1) = -a2*(sq1*ctb + stb*cq1)*sq2 - a3*(sq1*ctb + stb*cq1)*sq2*cq3 - a3*(sq1*ctb + stb*cq1)*sq3*cq2 + d5*(((-sq1*ctb - stb*cq1)*sq2*sq3 - (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*sq4) + d6*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*sq5 + d7*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*sq5;
    // Ja(1,2) = -a3*(sq1*ctb + stb*cq1)*sq2*cq3 - a3*(sq1*ctb + stb*cq1)*sq3*cq2 + d5*(((-sq1*ctb - stb*cq1)*sq2*sq3 - (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4) + d6*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5 + d7*(-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5;
    // Ja(1,3) = d5*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4) + d6*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5 + d7*((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5;
    // Ja(1,4) = d6*((-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (sq1*stb - cq1*ctb)*sq5) + d7*((-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (sq1*stb - cq1*ctb)*sq5);
    // Ja(1,5) = 0;
    //
    // Ja(2,0) = 0;
    // Ja(2,1) = a2*cq2 - a3*sq2*sq3 + a3*cq2*cq3 + d5*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5 - d7*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5;
    // Ja(2,2) = -a3*sq2*sq3 + a3*cq2*cq3 + d5*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5 - d7*((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*sq5;
    // Ja(2,3) = d5*(-(sq2*sq3 - cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4) - d6*((-sq2*sq3 + cq2*cq3)*cq4 - (sq2*cq3 + sq3*cq2)*sq4)*sq5 - d7*((-sq2*sq3 + cq2*cq3)*cq4 - (sq2*cq3 + sq3*cq2)*sq4)*sq5;
    // Ja(2,4) = -d6*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*cq5 - d7*((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*cq5;
    // Ja(2,5) = 0;
    //
    // Ja(3,0) = sq1*ctb + stb*cq1;
    // Ja(3,1) = sq1*ctb + stb*cq1;
    // Ja(3,2) = sq1*ctb + stb*cq1;
    // Ja(3,3) = (-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4;
    // Ja(3,4) = -((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5;
    // Ja(3,5) = -((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5;
    //
    // Ja(4,0) = sq1*stb - cq1*ctb;
    // Ja(4,1) = sq1*stb - cq1*ctb;
    // Ja(4,2) = sq1*stb - cq1*ctb;
    // Ja(4,3) = (-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4;
    // Ja(4,4) = -((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5;
    // Ja(4,5) = -((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5;
    //
    // Ja(5,0) = 0;
    // Ja(5,1) = 0;
    // Ja(5,2) = 0;
    // Ja(5,3) = -(-sq2*sq3 + cq2*cq3)*cq4 + (sq2*cq3 + sq3*cq2)*sq4;
    // Ja(5,4) = -((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5;
    // Ja(5,5) = -((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5;
    //
    // // Base Jacobian Jb
    // double Jb02 = a2*(-sq1*ctb - stb*cq1)*cq2 - a3*(-sq1*ctb - stb*cq1)*sq2*sq3 + a3*(-sq1*ctb - stb*cq1)*cq2*cq3 + d4*(-sq1*stb + cq1*ctb) + d5*(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4) + d6*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5) + d7*((-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5) - px*stb - py*ctb;
    //
    // double Jb12 = a2*(-sq1*stb + cq1*ctb)*cq2 - a3*(-sq1*stb + cq1*ctb)*sq2*sq3 + a3*(-sq1*stb + cq1*ctb)*cq2*cq3 + d4*(sq1*ctb + stb*cq1) + d5*(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*cq4) + d6*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5) + d7*((-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5) + px*ctb - py*stb;
    //
    // Jb << 1, 0, Jb02,
    //       0, 1, Jb12,
    //       0, 0, 0,
    //       0, 0, 0,
    //       0, 0, 0,
    //       0, 0, 1;
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
    Eigen::Matrix2d R_wb;
    double c = cos(q(2));
    double s = sin(q(2));
    R_wb << c, -s, s, c;

    // TODO this needs to be tested to see if it improves things.
    B = JointMatrix::Identity();
    // B.topLeftCorner<2, 3>() << R_wb, Eigen::Vector2d(-q(1), q(0));
    B.topLeftCorner<2, 2>() = R_wb;
}

} // namespace mm

