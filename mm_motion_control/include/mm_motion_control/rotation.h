#pragma once

#include <Eigen/Eigen>

#include <mm_kinematics/kinematics.h>
#include <mm_math_util/linalg.h>


namespace mm {

inline void rotation_error_jacobians(const JointVector& q,
                              Matrix3x9& Jn, Matrix3x9& Js, Matrix3x9& Ja) {
    double stb = std::sin(q(2));
    double ctb = std::cos(q(2));

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

Jn(0,0) = 0;
Jn(0,1) = 0;
Jn(0,2) = ((((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 + ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 + (-sq1*stb + cq1*ctb)*sq5)*cq6 + ((-(-sq1*ctb - stb*cq1)*cq2*cq3 - (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq6;
Jn(0,3) = ((((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 + ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 + (-sq1*stb + cq1*ctb)*sq5)*cq6 + ((-(-sq1*ctb - stb*cq1)*cq2*cq3 - (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq6;
Jn(0,4) = ((-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 + (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*cq5*cq6 + (((-sq1*stb + cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*cq4)*sq6;
Jn(0,5) = ((-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 + (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*cq5*cq6 + (((-sq1*stb + cq1*ctb)*sq3*cq2 - (sq1*stb - cq1*ctb)*sq2*cq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*cq4)*sq6;
Jn(0,6) = (-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*cq5*cq6 + (((-sq1*stb + cq1*ctb)*sq2*sq3 - (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq6;
Jn(0,7) = (-((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5)*cq6;
Jn(0,8) = -(((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 + (sq1*ctb + stb*cq1)*sq5)*sq6 + (-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*cq6;
Jn(1,0) = 0;
Jn(1,1) = 0;
Jn(1,2) = ((((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 + ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*cq5 + (sq1*ctb + stb*cq1)*sq5)*cq6 + ((-(-sq1*stb + cq1*ctb)*cq2*cq3 - (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4)*sq6;
Jn(1,3) = ((((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 + ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*cq5 + (sq1*ctb + stb*cq1)*sq5)*cq6 + ((-(-sq1*stb + cq1*ctb)*cq2*cq3 - (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4)*sq6;
Jn(1,4) = ((-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 + ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*cq5*cq6 + ((-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + (-(-sq1*ctb - stb*cq1)*sq3*cq2 + (sq1*ctb + stb*cq1)*sq2*cq3)*sq4)*sq6;
Jn(1,5) = ((-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 + ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*cq5*cq6 + ((-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + (-(-sq1*ctb - stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq6;
Jn(1,6) = (-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*cq5*cq6 + (((sq1*ctb + stb*cq1)*sq2*sq3 - (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq6;
Jn(1,7) = (-((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5)*cq6;
Jn(1,8) = -(((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 + (sq1*stb - cq1*ctb)*sq5)*sq6 + (-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*cq6;
Jn(2,0) = 0;
Jn(2,1) = 0;
Jn(2,2) = 0;
Jn(2,3) = 0;
Jn(2,4) = ((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*cq5*cq6 + ((sq2*sq3 - cq2*cq3)*sq4 + (-sq2*cq3 - sq3*cq2)*cq4)*sq6;
Jn(2,5) = ((-sq2*sq3 + cq2*cq3)*cq4 + (-sq2*cq3 - sq3*cq2)*sq4)*cq5*cq6 + ((sq2*sq3 - cq2*cq3)*sq4 + (-sq2*cq3 - sq3*cq2)*cq4)*sq6;
Jn(2,6) = (-(-sq2*sq3 + cq2*cq3)*sq4 + (-sq2*cq3 - sq3*cq2)*cq4)*sq6 + ((-sq2*sq3 + cq2*cq3)*cq4 - (sq2*cq3 + sq3*cq2)*sq4)*cq5*cq6;
Jn(2,7) = -((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5*cq6;
Jn(2,8) = -((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq6*cq5 + ((-sq2*sq3 + cq2*cq3)*cq4 - (sq2*cq3 + sq3*cq2)*sq4)*cq6;

Js(0,0) = 0;
Js(0,1) = 0;
Js(0,2) = (-(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 + ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (-sq1*stb + cq1*ctb)*sq5)*sq6 + ((-(-sq1*ctb - stb*cq1)*cq2*cq3 - (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*cq6;
Js(0,3) = (-(((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 + ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (-sq1*stb + cq1*ctb)*sq5)*sq6 + ((-(-sq1*ctb - stb*cq1)*cq2*cq3 - (sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*cq6;
Js(0,4) = -((-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 + (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq6*cq5 + (((-sq1*stb + cq1*ctb)*sq2*cq3 - (sq1*stb - cq1*ctb)*sq3*cq2)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*cq4)*cq6;
Js(0,5) = -((-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 + (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq6*cq5 + (((-sq1*stb + cq1*ctb)*sq3*cq2 - (sq1*stb - cq1*ctb)*sq2*cq3)*sq4 + (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*cq4)*cq6;
Js(0,6) = -(-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq6*cq5 + (((-sq1*stb + cq1*ctb)*sq2*sq3 - (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq6;
Js(0,7) = (((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5 - (sq1*ctb + stb*cq1)*cq5)*sq6;
Js(0,8) = (-((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5)*cq6 - (-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq6;
Js(1,0) = 0;
Js(1,1) = 0;
Js(1,2) = (-(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 + ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5)*sq6 + ((-(-sq1*stb + cq1*ctb)*cq2*cq3 - (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4)*cq6;
Js(1,3) = (-(((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 + ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5)*sq6 + ((-(-sq1*stb + cq1*ctb)*cq2*cq3 - (sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4)*cq6;
Js(1,4) = -((-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 + ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*sq6*cq5 + ((-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + (-(-sq1*ctb - stb*cq1)*sq3*cq2 + (sq1*ctb + stb*cq1)*sq2*cq3)*sq4)*cq6;
Js(1,5) = -((-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 + ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq6*cq5 + ((-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + (-(-sq1*ctb - stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq6;
Js(1,6) = -(-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq6*cq5 + (((sq1*ctb + stb*cq1)*sq2*sq3 - (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq6;
Js(1,7) = (((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 - (sq1*stb - cq1*ctb)*cq5)*sq6;
Js(1,8) = (-((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (sq1*stb - cq1*ctb)*sq5)*cq6 - (-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq6;
Js(2,0) = 0;
Js(2,1) = 0;
Js(2,2) = 0;
Js(2,3) = 0;
Js(2,4) = (-(-sq2*sq3 + cq2*cq3)*cq4 - (-sq2*cq3 - sq3*cq2)*sq4)*sq6*cq5 + ((sq2*sq3 - cq2*cq3)*sq4 + (-sq2*cq3 - sq3*cq2)*cq4)*cq6;
Js(2,5) = (-(-sq2*sq3 + cq2*cq3)*cq4 - (-sq2*cq3 - sq3*cq2)*sq4)*sq6*cq5 + ((sq2*sq3 - cq2*cq3)*sq4 + (-sq2*cq3 - sq3*cq2)*cq4)*cq6;
Js(2,6) = (-(-sq2*sq3 + cq2*cq3)*sq4 + (-sq2*cq3 - sq3*cq2)*cq4)*cq6 + (-(-sq2*sq3 + cq2*cq3)*cq4 + (sq2*cq3 + sq3*cq2)*sq4)*sq6*cq5;
Js(2,7) = -(-(-sq2*sq3 + cq2*cq3)*sq4 - (sq2*cq3 + sq3*cq2)*cq4)*sq5*sq6;
Js(2,8) = (-(-sq2*sq3 + cq2*cq3)*sq4 - (sq2*cq3 + sq3*cq2)*cq4)*cq5*cq6 - ((-sq2*sq3 + cq2*cq3)*cq4 - (sq2*cq3 + sq3*cq2)*sq4)*sq6;

Ja(0,0) = 0;
Ja(0,1) = 0;
Ja(0,2) = (-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5;
Ja(0,3) = (-((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - ((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-sq1*stb + cq1*ctb)*cq5;
Ja(0,4) = (-(-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5;
Ja(0,5) = (-(-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 - (-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5;
Ja(0,6) = ((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq5;
Ja(0,7) = (-(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 - (sq1*ctb + stb*cq1)*sq5;
Ja(0,8) = 0;
Ja(1,0) = 0;
Ja(1,1) = 0;
Ja(1,2) = (-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5;
Ja(1,3) = (-((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - ((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5;
Ja(1,4) = (-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*sq5;
Ja(1,5) = (-(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - ((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5;
Ja(1,6) = ((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5;
Ja(1,7) = (-(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (sq1*stb - cq1*ctb)*sq5;
Ja(1,8) = 0;
Ja(2,0) = 0;
Ja(2,1) = 0;
Ja(2,2) = 0;
Ja(2,3) = 0;
Ja(2,4) = (-(-sq2*sq3 + cq2*cq3)*cq4 - (-sq2*cq3 - sq3*cq2)*sq4)*sq5;
Ja(2,5) = (-(-sq2*sq3 + cq2*cq3)*cq4 - (-sq2*cq3 - sq3*cq2)*sq4)*sq5;
Ja(2,6) = (-(-sq2*sq3 + cq2*cq3)*cq4 + (sq2*cq3 + sq3*cq2)*sq4)*sq5;
Ja(2,7) = (-(-sq2*sq3 + cq2*cq3)*sq4 - (sq2*cq3 + sq3*cq2)*cq4)*cq5;
Ja(2,8) = 0;
}


// Calculate rotation error
// d: desired orientation
// q: current joint positions
// e: populated with orientation error vector
inline void rotation_error(const Eigen::Quaterniond& d, const JointVector& q,
                    Eigen::Vector3d& e) {
    // Desired EE rotation.
    Eigen::Matrix3d Rd = d.toRotationMatrix();

    // Current EE rotation (extract from current pose).
    Eigen::Affine3d w_T_e;
    Kinematics::calc_w_T_e(q, w_T_e);
    Eigen::Matrix3d Re = w_T_e.rotation();

    Eigen::Vector3d nd = Rd.col(0);
    Eigen::Vector3d sd = Rd.col(1);
    Eigen::Vector3d ad = Rd.col(2);

    Eigen::Vector3d ne = Re.col(0);
    Eigen::Vector3d se = Re.col(1);
    Eigen::Vector3d ae = Re.col(2);

    e = 0.5 * (ne.cross(nd) + se.cross(sd) + ae.cross(ad));
}


inline void linearize_rotation_error(const Eigen::Quaterniond& d, const JointVector& q,
                              double dt, Eigen::Vector3d& e, Matrix3x9& J) {
    // Desired EE rotation.
    Eigen::Matrix3d Rd = d.toRotationMatrix();

    // Current EE rotation (extract from current pose).
    Eigen::Affine3d w_T_e;
    Kinematics::calc_w_T_e(q, w_T_e);
    Eigen::Matrix3d Re = w_T_e.rotation();

    // Skew matrices to take the cross product.
    Eigen::Matrix3d Nd = skew(Rd.col(0));
    Eigen::Matrix3d Sd = skew(Rd.col(1));
    Eigen::Matrix3d Ad = skew(Rd.col(2));

    Eigen::Vector3d ne = Re.col(0);
    Eigen::Vector3d se = Re.col(1);
    Eigen::Vector3d ae = Re.col(2);

    // e = r * sin(phi), where (r, phi) are the (axis, angle) representing the
    // orientation error. See Siciliano et al., 2010, pg. 139 for details.
    // Negative sign is the result of reversing the order of the cross products.
    e = -0.5 * (Nd * ne + Sd * se + Ad * ae);

    // Construct Jacobian of e for first-order linearization.
    Matrix3x9 Jn, Js, Ja;
    rotation_error_jacobians(q, Jn, Js, Ja);
    J = -0.5 * dt * (Nd * Jn + Sd * Js + Ad * Ja);
}

inline void linearize_position_error(const Eigen::Vector3d& d, const JointVector& q,
                              double dt, Eigen::Vector3d& e, Matrix3x9& J) {

    // Calculate actual pose using forward kinematics.
    Eigen::Affine3d w_T_e;
    Kinematics::calc_w_T_e(q, w_T_e);
    Eigen::Vector3d p = w_T_e.translation();
    e = d - p;

    // Jacobian for position error is just the top three rows of the usual
    // geometric Jacobian.
    JacobianMatrix J_geo;
    Kinematics::jacobian(q, J_geo);
    J = -dt * J_geo.topRows<3>();
}

} // namespace mm
