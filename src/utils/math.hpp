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
