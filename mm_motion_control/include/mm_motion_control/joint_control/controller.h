#pragma once

#include <mm_kinematics/kinematics.h>


namespace mm {

// Proportional joint-space controller.
class JointController {
    public:
        JointController() {}

        // Initialize the controller.
        // Parameters:
        //   K: proportional gain matrix
        //
        // Returns:
        //   true if controller should continue, false otherwise
        bool init(const JointMatrix& K);

        // Run one control iteration.
        // Parameters:
        //   d: desired joint positions
        //   a: actual joint positions
        //   u: populated joint velocity
        //
        // Returns:
        //   true if the update was successful, false otherwise
        bool update(const JointVector& d, const JointVector& a, JointVector& u);

    private:
        JointMatrix K;
}; // class JointController

} // namespace mm
