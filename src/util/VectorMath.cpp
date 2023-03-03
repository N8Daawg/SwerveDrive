#include "util/VectorMath.hpp"


double magnitude(cartesian_vector v) {
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}


cartesian_vector add_vectors(cartesian_vector v1, cartesian_vector v2) {
    return {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
}


cartesian_vector diff_vectors(cartesian_vector v1, cartesian_vector v2) {
    return {v1.x - v2.x, v1.y - v2.y, v1.z - v2.z};
}


cartesian_vector cross_product(cartesian_vector v1, cartesian_vector v2) {
    double x = (v1.y * v2.z) - (v1.z * v2.y);
    double y = -((v1.x * v2.z) - (v1.z * v2.x));
    double z = (v1.x * v2.y) - (v1.y * v2.x);
    return {x, y, z};
}


double dot_product(cartesian_vector v1, cartesian_vector v2) {
    return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z);
}
