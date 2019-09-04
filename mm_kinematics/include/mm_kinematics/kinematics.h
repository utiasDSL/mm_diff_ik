#pragma once

#include <ros/ros.h>
#include <Eigen/Eigen>


#define DH_TF(T, q, a, d, alpha) Eigen::Affine3d T; dh_transform(q, a, d, alpha, T);


namespace mm {

const uint32_t NUM_BASE_JOINTS = 3;
const uint32_t NUM_ARM_JOINTS = 6;
const uint32_t NUM_JOINTS = NUM_BASE_JOINTS + NUM_ARM_JOINTS;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

typedef Eigen::Matrix<double, NUM_ARM_JOINTS,  1>               ArmJointVector;
typedef Eigen::Matrix<double, NUM_ARM_JOINTS,  NUM_ARM_JOINTS>  ArmJointMatrix;
typedef Eigen::Matrix<double, NUM_BASE_JOINTS, 1>               BaseJointVector;
typedef Eigen::Matrix<double, NUM_BASE_JOINTS, NUM_BASE_JOINTS> BaseJointMatrix;
typedef Eigen::Matrix<double, NUM_JOINTS,      1>               JointVector;
typedef Eigen::Matrix<double, NUM_JOINTS,      NUM_JOINTS>      JointMatrix;

typedef Eigen::Matrix<double, 6, NUM_BASE_JOINTS> BaseJacobianMatrix;
typedef Eigen::Matrix<double, 6, NUM_ARM_JOINTS>  ArmJacobianMatrix;
typedef Eigen::Matrix<double, 6, NUM_JOINTS>      JacobianMatrix;


// Joint position limits
// base: -0.75pi : 0.75pi
// shoulder: -1.25pi : -0.25pi
// elbow: -pi : pi
// w1: -2pi : 2pi
// w2: -2pi : 2pi
// w3: -2pi : 2pi
static const double TWO_PI = 2.0*M_PI;
const JointVector POSITION_LIMITS_LOWER(
        (JointVector() << -4.0, -4.0, -M_PI,
                          -0.75*M_PI, -1.25*M_PI, -M_PI, -TWO_PI, -TWO_PI, -TWO_PI).finished());
const JointVector POSITION_LIMITS_UPPER(
        (JointVector() << 4.0, 4.0, M_PI,
                          0.75*M_PI, -0.25*M_PI, M_PI, TWO_PI, TWO_PI, TWO_PI).finished());

// Joint velocity limits
const JointVector VELOCITY_LIMITS_UPPER(
        (JointVector() << 1.0, 1.0, 2.0, 2.16, 2.16, 3.15, 3.2, 3.2, 3.2).finished());
const JointVector VELOCITY_LIMITS_LOWER = -VELOCITY_LIMITS_UPPER;

const std::string JOINT_NAMES[] = { "qx", "qy", "qt", "q1", "q2", "q3", "q4", "q5", "q6" };


class Kinematics {
    public:
        // Calculate Jacobian
        // q: current joint values
        // J: populated with Jacobian matrix
        static void jacobian(const JointVector& q, JacobianMatrix& J);

        // Forward kinematics
        // q: joint values
        // P: populated with pose of end effector
        static void forward(const JointVector& q, Eigen::Affine3d& w_T_e);

        // Transform from base to world.
        static void calc_w_T_b(const JointVector& q, Eigen::Affine3d& w_T_b);

        // Transform from arm to world.
        static void calc_w_T_a(const JointVector& q, Eigen::Affine3d& w_T_a);

        // Transform from EE to world.
        static void calc_w_T_e(const JointVector& q, Eigen::Affine3d& w_T_e);

        // Forward velocity kinematics.
        static void forward_vel(const JointVector& q,
                                const JointVector& dq, Vector6d& v);

        // Manipulability index.
        static double manipulability(const JointVector& q);

    private:
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

        // Create transformation matrix from D-H parameters.
        static void dh_transform(double q, double a, double d, double alpha,
                                 Eigen::Affine3d& T);

        static void jacobians(const JointVector& q, ArmJacobianMatrix& Ja,
                              BaseJacobianMatrix& Jb);
}; // class Kinematics

} // namespace mm

