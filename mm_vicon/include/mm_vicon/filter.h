#pragma once

#include <ros/ros.h>


namespace mm {

// First-order exponential smoothing: https://en.wikipedia.org/wiki/Exponential_smoothing
template <class T>
class ExponentialSmoother {
    public:
        ExponentialSmoother() {}

        // Initialize with time constant tau and initial value x0. As tau
        // increases, new measurements are trusted less and more weight is
        // given to the old states.
        //
        // Take care setting x0 since it can heavily weight early
        // estimates depending on the magnitude of tau.
        void init(double tau, const T& x0);

        // Generate next estimate from measurement.
        T next(const T& measured, double dt);

    private:
        T prev; // previous measurement
        double tau; // time constant
}; // class ExponentialSmoother

} // namespace mm

// include implementation
#include "mm_vicon/filter_impl.h"
