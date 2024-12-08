#ifndef UTILS_MATH_H
#define UTILS_MATH_H

#include <boost/container_hash/hash.hpp>

#include <cmath>
#include <limits>

/**
 * a == b
 */
template<typename T>
bool is_close_equal_to(T a, T b, T epsilon = std::numeric_limits<T>::epsilon()) {
    return fabs(a - b) < epsilon;
}


/**
 * a >= b
 */
template<typename T>
bool is_close_no_less_than(T a, T b, T epsilon = std::numeric_limits<T>::epsilon()) {
    return a > b || is_close_equal_to(a, b, epsilon);
}


/**
 * a <= b
 */
template<typename T>
bool is_close_no_greater_than(T a, T b, T epsilon = std::numeric_limits<T>::epsilon()) {
    return a < b || is_close_equal_to(a, b, epsilon);
}


static size_t combineRandomSeed(unsigned int nodeId1, unsigned int nodeId2, unsigned int timestep, unsigned int _seed) {
    size_t result = 0;
    boost::hash_combine(result, nodeId1 ^ nodeId2);
    boost::hash_combine(result, timestep);
    boost::hash_combine(result, _seed);
    return result;
}

#endif // UTILS_MATH_H