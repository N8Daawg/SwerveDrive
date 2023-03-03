// VectorMath.hpp
// Contains classes and methods for dealing with cartesian vectors
//
// Author: Aiden Carney

#ifndef VECTORMATH_HPP
#define VECTORMATH_HPP

#include <cmath>


typedef struct {
    double x;
    double y;
    double z;
} cartesian_vector;


// Calculates the magnitude of a vector
//
// Params:
//    v - the vector to take the magnitude of
// Return:
//    double - the magnitude
double magnitude(cartesian_vector v);


// Adds two vectors
//
// Params:
//    v1 - the first vector
//    v2 - the second vector
// Return:
//    cartesian_vector - the new vector
cartesian_vector add_vectors(cartesian_vector v1, cartesian_vector v2);


// Subtracts two vectors
//
// Params:
//    v1 - the first vector
//    v2 - the second vector
// Return:
//    cartesian_vector - the new vector
cartesian_vector diff_vectors(cartesian_vector v1, cartesian_vector v2);


// takes the cross product of two vectors
//
// Params:
//    v1 - the first vector
//    v2 - the second vector
// Return:
//    cartesian_vector - the new vector
cartesian_vector cross_product(cartesian_vector v1, cartesian_vector v2);


// takes the dot product of two vectors
//
// Params:
//    v1 - the first vector
//    v2 - the second vector
// Return:
//    double - the calculated dot product
double dot_product(cartesian_vector v1, cartesian_vector v2);



#endif
