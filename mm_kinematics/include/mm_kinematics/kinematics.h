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

typedef Eigen::Matrix<double, NUM_ARM_JOINTS,  NUM_ARM_JOINTS>  ArmJointMatrix;
typedef Eigen::Matrix<double, NUM_BASE_JOINTS, NUM_BASE_JOINTS> BaseJointMatrix;
typedef Eigen::Matrix<double, NUM_JOINTS,      NUM_JOINTS>      JointMatrix;

typedef Eigen::Matrix<double, NUM_ARM_JOINTS,  1> ArmJointVector;
typedef Eigen::Matrix<double, NUM_BASE_JOINTS, 1> BaseJointVector;
typedef Eigen::Matrix<double, NUM_JOINTS,      1> JointVector;

typedef Eigen::Matrix<double, 6, NUM_BASE_JOINTS> BaseJacobianMatrix;
typedef Eigen::Matrix<double, 6, NUM_ARM_JOINTS>  ArmJacobianMatrix;
typedef Eigen::Matrix<double, 6, NUM_JOINTS>      JacobianMatrix;

typedef Eigen::Matrix<double, 3, 9> Matrix3x9;


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
// const JointVector VELOCITY_LIMITS_UPPER(
//         (JointVector() << 1.0, 1.0, 2.0, 2.16, 2.16, 3.15, 3.2, 3.2, 3.2).finished());
// NOTE: conservative velocity limits until mysterious controller instability
// bug is resolved.
const JointVector VELOCITY_LIMITS_UPPER(
        (JointVector() << 0.5, 0.5, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25).finished());
const JointVector VELOCITY_LIMITS_LOWER = -VELOCITY_LIMITS_UPPER;

const JointVector ACCEL_LIMITS_UPPER(
        (JointVector() << 1.0, 1.0, 1.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0).finished());
const JointVector ACCEL_LIMITS_LOWER = -ACCEL_LIMITS_UPPER;

const std::string JOINT_NAMES[] = { "qx", "qy", "qt", "q1", "q2", "q3", "q4", "q5", "q6" };

const double BASE_RADIUS = 0.5; // m


class Kinematics {
    public:
        // Calculate Jacobian
        // q: current joint positions
        // J: populated with Jacobian matrix
        static void jacobian(const JointVector& q, JacobianMatrix& J);

        // Transform from mobile base to world.
        static void calc_w_T_base(const JointVector& q,
                                  Eigen::Affine3d& w_T_base);

        // Transform from base of arm to world.
        static void calc_w_T_arm(const JointVector& q,
                                 Eigen::Affine3d& w_T_arm);

        // Transform from EE to world.
        static void calc_w_T_ee(const JointVector& q, Eigen::Affine3d& w_T_ee);

        // Transform from tool to world.
        static void calc_w_T_tool(const JointVector& q,
                                  Eigen::Affine3d& w_T_tool);

        // Manipulability index.
        static double manipulability(const JointVector& q);

        static void calc_base_input_mapping(const JointVector& q,
                                            JointMatrix& B);

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
        static constexpr double d7 = 0.290;

        // Create transformation matrix from D-H parameters.
        static void dh_transform(double q, double a, double d, double alpha,
                                 Eigen::Affine3d& T);

        static void jacobians(const JointVector& q, ArmJacobianMatrix& Ja,
                              BaseJacobianMatrix& Jb);
}; // class Kinematics

} // namespace mm

