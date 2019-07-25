#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>

#include "rr/rr.h"


#define DH_TF(T, q, a, d, alpha) Affine3d T; dh_transform(q, a, d, alpha, T);

using namespace Eigen;

namespace rr {
    class Kinematics {
        public:

        // Calculate Jacoian
        // q: current joint values
        // J: populated with Jacobian matrix
        static void jacobian(const QVector& q, JacMatrix& J) {
            // Base joints
            double xb = q[0];
            double yb = q[1];
            double stb = std::sin(q[2]);
            double ctb = std::cos(q[2]);

            // Arm joints
            double sq1 = std::sin(q[3]);
            double cq1 = std::cos(q[3]);
            double sq2 = std::sin(q[4]);
            double cq2 = std::cos(q[4]);
            double sq3 = std::sin(q[5]);
            double cq3 = std::cos(q[5]);
            double sq4 = std::sin(q[6]);
            double cq4 = std::cos(q[6]);
            double sq5 = std::sin(q[7]);
            double cq5 = std::cos(q[7]);
            double sq6 = std::sin(q[8]);
            double cq6 = std::cos(q[8]);

            // here be dragons - generated from the `thing_kinematics.py` Python script
            // using symbolic math
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

        // Forward kinematics
        // q: joint values
        // P: populated with pose of end effector
        static void forward(const QVector& q, Affine3d& w_T_e) {
            // Base
            DH_TF(T1, M_PI_2, 0, 0,    M_PI_2);
            DH_TF(T2, M_PI_2, 0, q(0), M_PI_2);
            DH_TF(T3, M_PI_2, 0, q(1), M_PI_2);
            DH_TF(T4, q(2),   0, 0,    0);

            Affine3d w_T_b = T1 * T2 * T3 * T4;

            // between base and arm
            DH_TF(T5, 0, 0.27, 0.653, -M_PI_2);
            DH_TF(T6, 0, 0,    0.01,   M_PI_2);

            Affine3d b_T_a = T5 * T6;

            // Arm
            DH_TF(T7,  q(3),  0,      0.1273,   M_PI_2);
            DH_TF(T8,  q(4), -0.612,  0,        0);
            DH_TF(T9,  q(5), -0.5723, 0,        0);
            DH_TF(T10, q(6),  0,      0.163941, M_PI_2);
            DH_TF(T11, q(7),  0,      0.1157,  -M_PI_2);
            DH_TF(T12, q(8),  0,      0.0922,   0);

            Affine3d a_T_e = T7 * T8 * T9 * T10 * T11 * T12;

            w_T_e = w_T_b * b_T_a * a_T_e;
        }

        private:

        // Create transformation matrix from D-H parameters.
        static void dh_transform(double q, double a, double d, double alpha,
                                 Affine3d& T) {
            double sq = std::sin(q);
            double cq = std::cos(q);
            double salpha = std::sin(alpha);
            double calpha = std::cos(alpha);

            T.matrix() << cq, -sq*calpha, sq*salpha, a*cq,
                          sq,  cq*calpha, cq*salpha, a*sq,
                          0,   salpha,    calpha,    d,
                          0,   0,         0,         1;
        }

    };
}

