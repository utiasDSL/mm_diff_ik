#include "mm_vicon/filter.h"


namespace mm {
    template<class T>
    void ExponentialSmoother<T>::init(double tau, const T& x0) {
        this->tau = tau;
        prev = x0;
    }

    template<class T>
    T ExponentialSmoother<T>::next(const T& measured, double dt) {
        double c = 1.0 - std::exp(-dt / tau);
        T state = c * measured + (1 - c) * prev;
        prev = state;
        return state;
    }
}
