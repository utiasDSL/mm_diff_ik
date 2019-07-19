#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>

#include "rr/rr.h"

namespace rr {

void calc_jacobian(QVector& q_current, JacMatrix& J) {
    // Base joints
    double xb = q_current[0];
    double yb = q_current[1];
    double stb = std::sin(q_current[2]);
    double ctb = std::cos(q_current[2]);

    // Arm joints
    double sq1 = std::sin(q_current[3]);
    double cq1 = std::cos(q_current[3]);
    double sq2 = std::sin(q_current[4]);
    double cq2 = std::cos(q_current[4]);
    double sq3 = std::sin(q_current[5]);
    double cq3 = std::cos(q_current[5]);
    double sq4 = std::sin(q_current[6]);
    double cq4 = std::cos(q_current[6]);
    double sq5 = std::sin(q_current[7]);
    double cq5 = std::cos(q_current[7]);
    double sq6 = std::sin(q_current[8]);
    double cq6 = std::cos(q_current[8]);

    // here be dragons
    J << 1,
0,
(-0.0922*((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - 0.0922*((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-0.0922*sq1*stb + 0.0922*cq1*ctb)*cq5 + (-0.5723*sq1*ctb - 0.5723*stb*cq1)*sq2*sq3 + (0.5723*sq1*ctb + 0.5723*stb*cq1)*cq2*cq3 + (0.612*sq1*ctb + 0.612*stb*cq1)*cq2 + (0.1157*(-sq1*ctb - stb*cq1)*cq2*cq3 + 0.1157*(sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-0.1157*(sq1*ctb + stb*cq1)*sq2*cq3 - 0.1157*(sq1*ctb + stb*cq1)*sq3*cq2)*cq4 - 0.163941*sq1*stb - 0.27*stb + 0.163941*cq1*ctb - 0.01*ctb,
(-0.0922*((-sq1*ctb - stb*cq1)*cq2*cq3 + (sq1*ctb + stb*cq1)*sq2*sq3)*cq4 - 0.0922*((sq1*ctb + stb*cq1)*sq2*cq3 + (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (-0.0922*sq1*stb + 0.0922*cq1*ctb)*cq5 + (-0.5723*sq1*ctb - 0.5723*stb*cq1)*sq2*sq3 + (0.5723*sq1*ctb + 0.5723*stb*cq1)*cq2*cq3 + (0.612*sq1*ctb + 0.612*stb*cq1)*cq2 + (0.1157*(-sq1*ctb - stb*cq1)*cq2*cq3 + 0.1157*(sq1*ctb + stb*cq1)*sq2*sq3)*sq4 + (-0.1157*(sq1*ctb + stb*cq1)*sq2*cq3 - 0.1157*(sq1*ctb + stb*cq1)*sq3*cq2)*cq4 - 0.163941*sq1*stb + 0.163941*cq1*ctb,
(-0.0922*(-(-sq1*stb + cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*cq4 - 0.0922*(-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5 + (-0.5723*sq1*stb + 0.5723*cq1*ctb)*sq3*cq2 - (0.5723*sq1*stb - 0.5723*cq1*ctb)*sq2*cq3 - (0.612*sq1*stb - 0.612*cq1*ctb)*sq2 + (-0.1157*(-sq1*stb + cq1*ctb)*sq2*cq3 + 0.1157*(sq1*stb - cq1*ctb)*sq3*cq2)*sq4 + (0.1157*(sq1*stb - cq1*ctb)*sq2*sq3 - 0.1157*(sq1*stb - cq1*ctb)*cq2*cq3)*cq4,
(-0.0922*(-(-sq1*stb + cq1*ctb)*sq3*cq2 + (sq1*stb - cq1*ctb)*sq2*cq3)*cq4 - 0.0922*(-(sq1*stb - cq1*ctb)*sq2*sq3 + (sq1*stb - cq1*ctb)*cq2*cq3)*sq4)*sq5 + (-0.5723*sq1*stb + 0.5723*cq1*ctb)*sq2*cq3 - (0.5723*sq1*stb - 0.5723*cq1*ctb)*sq3*cq2 + (-0.1157*(-sq1*stb + cq1*ctb)*sq3*cq2 + 0.1157*(sq1*stb - cq1*ctb)*sq2*cq3)*sq4 + (0.1157*(sq1*stb - cq1*ctb)*sq2*sq3 - 0.1157*(sq1*stb - cq1*ctb)*cq2*cq3)*cq4,
(0.0922*(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - 0.0922*(-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4)*sq5 + (-0.1157*(-sq1*stb + cq1*ctb)*sq2*sq3 + 0.1157*(-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - (0.1157*(-sq1*stb + cq1*ctb)*sq2*cq3 + 0.1157*(-sq1*stb + cq1*ctb)*sq3*cq2)*sq4,
(-0.0922*(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 - 0.0922*(-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*cq5 - (0.0922*sq1*ctb + 0.0922*stb*cq1)*sq5,
0,
0,
1,
(-0.0922*((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - 0.0922*((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (-0.5723*sq1*stb + 0.5723*cq1*ctb)*sq2*sq3 + (0.5723*sq1*stb - 0.5723*cq1*ctb)*cq2*cq3 + (0.612*sq1*stb - 0.612*cq1*ctb)*cq2 + (0.0922*sq1*ctb + 0.0922*stb*cq1)*cq5 + (0.1157*(-sq1*stb + cq1*ctb)*cq2*cq3 + 0.1157*(sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-0.1157*(sq1*stb - cq1*ctb)*sq2*cq3 - 0.1157*(sq1*stb - cq1*ctb)*sq3*cq2)*cq4 + 0.163941*sq1*ctb + 0.163941*stb*cq1 - 0.01*stb + 0.27*ctb,
(-0.0922*((-sq1*stb + cq1*ctb)*cq2*cq3 + (sq1*stb - cq1*ctb)*sq2*sq3)*cq4 - 0.0922*((sq1*stb - cq1*ctb)*sq2*cq3 + (sq1*stb - cq1*ctb)*sq3*cq2)*sq4)*sq5 + (-0.5723*sq1*stb + 0.5723*cq1*ctb)*sq2*sq3 + (0.5723*sq1*stb - 0.5723*cq1*ctb)*cq2*cq3 + (0.612*sq1*stb - 0.612*cq1*ctb)*cq2 + (0.0922*sq1*ctb + 0.0922*stb*cq1)*cq5 + (0.1157*(-sq1*stb + cq1*ctb)*cq2*cq3 + 0.1157*(sq1*stb - cq1*ctb)*sq2*sq3)*sq4 + (-0.1157*(sq1*stb - cq1*ctb)*sq2*cq3 - 0.1157*(sq1*stb - cq1*ctb)*sq3*cq2)*cq4 + 0.163941*sq1*ctb + 0.163941*stb*cq1,
(-0.0922*(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - 0.0922*((-sq1*ctb - stb*cq1)*sq3*cq2 - (sq1*ctb + stb*cq1)*sq2*cq3)*cq4)*sq5 - (-0.612*sq1*ctb - 0.612*stb*cq1)*sq2 - (-0.5723*sq1*ctb - 0.5723*stb*cq1)*sq2*cq3 + (0.5723*sq1*ctb + 0.5723*stb*cq1)*sq3*cq2 + (0.1157*(-sq1*ctb - stb*cq1)*sq2*sq3 - 0.1157*(-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + (0.1157*(-sq1*ctb - stb*cq1)*sq3*cq2 - 0.1157*(sq1*ctb + stb*cq1)*sq2*cq3)*sq4,
(-0.0922*(-(-sq1*ctb - stb*cq1)*sq2*sq3 + (-sq1*ctb - stb*cq1)*cq2*cq3)*sq4 - 0.0922*((-sq1*ctb - stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5 - (-0.5723*sq1*ctb - 0.5723*stb*cq1)*sq3*cq2 + (0.5723*sq1*ctb + 0.5723*stb*cq1)*sq2*cq3 + (0.1157*(-sq1*ctb - stb*cq1)*sq2*sq3 - 0.1157*(-sq1*ctb - stb*cq1)*cq2*cq3)*cq4 + (0.1157*(-sq1*ctb - stb*cq1)*sq2*cq3 - 0.1157*(sq1*ctb + stb*cq1)*sq3*cq2)*sq4,
(0.0922*(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - 0.0922*(-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4)*sq5 + (-0.1157*(sq1*ctb + stb*cq1)*sq2*sq3 + 0.1157*(sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - (0.1157*(sq1*ctb + stb*cq1)*sq2*cq3 + 0.1157*(sq1*ctb + stb*cq1)*sq3*cq2)*sq4,
(-0.0922*(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 - 0.0922*(-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*cq5 - (0.0922*sq1*stb - 0.0922*cq1*ctb)*sq5,
0,
0,
0,
0,
0,
(-0.0922*(-sq2*sq3 + cq2*cq3)*cq4 - 0.0922*(-sq2*cq3 - sq3*cq2)*sq4)*sq5 + (-0.1157*sq2*sq3 + 0.1157*cq2*cq3)*sq4 + (0.1157*sq2*cq3 + 0.1157*sq3*cq2)*cq4 + 0.5723*sq2*sq3 - 0.5723*cq2*cq3 - 0.612*cq2,
(-0.0922*(-sq2*sq3 + cq2*cq3)*cq4 - 0.0922*(-sq2*cq3 - sq3*cq2)*sq4)*sq5 + (-0.1157*sq2*sq3 + 0.1157*cq2*cq3)*sq4 + (0.1157*sq2*cq3 + 0.1157*sq3*cq2)*cq4 + 0.5723*sq2*sq3 - 0.5723*cq2*cq3,
(-0.0922*(-sq2*sq3 + cq2*cq3)*cq4 + 0.0922*(sq2*cq3 + sq3*cq2)*sq4)*sq5 - (0.1157*sq2*sq3 - 0.1157*cq2*cq3)*sq4 + (0.1157*sq2*cq3 + 0.1157*sq3*cq2)*cq4,
(-0.0922*(-sq2*sq3 + cq2*cq3)*sq4 - 0.0922*(sq2*cq3 + sq3*cq2)*cq4)*cq5,
0,
0,
0,
0,
sq1*ctb + stb*cq1,
sq1*ctb + stb*cq1,
sq1*ctb + stb*cq1,
(-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*sq4 - (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*cq4,
-((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5,
-((-(-sq1*stb + cq1*ctb)*sq2*sq3 + (-sq1*stb + cq1*ctb)*cq2*cq3)*cq4 + (-(-sq1*stb + cq1*ctb)*sq2*cq3 - (-sq1*stb + cq1*ctb)*sq3*cq2)*sq4)*sq5 + (sq1*ctb + stb*cq1)*cq5,
0,
0,
0,
sq1*stb - cq1*ctb,
sq1*stb - cq1*ctb,
sq1*stb - cq1*ctb,
(-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*sq4 - (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*cq4,
-((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5,
-((-(sq1*ctb + stb*cq1)*sq2*sq3 + (sq1*ctb + stb*cq1)*cq2*cq3)*cq4 + (-(sq1*ctb + stb*cq1)*sq2*cq3 - (sq1*ctb + stb*cq1)*sq3*cq2)*sq4)*sq5 + (sq1*stb - cq1*ctb)*cq5,
0,
0,
1,
0,
0,
0,
-(-sq2*sq3 + cq2*cq3)*cq4 + (sq2*cq3 + sq3*cq2)*sq4,
-((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5,
-((-sq2*sq3 + cq2*cq3)*sq4 + (sq2*cq3 + sq3*cq2)*cq4)*sq5;

}
}
