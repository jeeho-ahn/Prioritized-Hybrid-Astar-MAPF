#ifndef UTILS_H
#define UTILS_H

#include <cmath>

// from OMPL
double twopi = 2. * M_PI;
const double DUBINS_EPS = 1e-6;
const double DUBINS_ZERO = -1e-7;
inline double mod2pi(double x) {
    if (x < 0 && x > DUBINS_ZERO) //DUBINS_ZERO
        return 0;
    double xm = x - twopi * floor(x / twopi);
    if (twopi - xm < .5 * DUBINS_EPS) //DUBINS_EPS
        xm = 0.;
    return xm;
}

inline double pi_2_pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


#endif // UTILS_H
