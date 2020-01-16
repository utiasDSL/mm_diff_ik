#include "mm_motion_control/joint_control/controller.h"


namespace mm {

bool JointController::init(const JointMatrix& K) {
    this->K = K;
    return true;
}


bool JointController::update(const JointVector& d, const JointVector& a, JointVector& u) {
    JointVector e = d - a;

    // If error is small enough, no need to do anything.
    if (e.isZero(1e-3)) {
        return false;
    }

    // Proportional control.
    u = K * e;

    return true;
}

} // namespace mm
