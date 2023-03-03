// misc.hpp
// Contains miscellaneous utility methods

#ifndef MISC_HPP
#define MISC_HPP

double scale(double n, double newMin, double newMax, double oldMin, double oldMax) {
    return ((newMax - newMin) * (n - oldMin) / (oldMax - oldMin)) + newMin;
}



#endif