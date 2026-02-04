#ifndef CORE_SATURATE_HPP
#define CORE_SATURATE_HPP


#include "core_Math.hpp"
#include <algorithm>

/**
 * @brief Saturate a scalar value between min and max bounds
 */
template<typename T>
T saturate(T value, T min_val, T max_val) {
    return std::max(min_val, std::min(value, max_val));
}

/**
 * @brief Element-wise saturation of a vector between min and max bounds
 */
template<int N>
Math::Vec<N> saturateVec(const Math::Vec<N>& value, 
                         const Math::Vec<N>& min_val, 
                         const Math::Vec<N>& max_val) {
    Math::Vec<N> result;
    for (int i = 0; i < N; ++i) {
        result(i) = saturate(value(i), min_val(i), max_val(i));
    }
    return result;
}

/**
 * @brief Saturate a vector with symmetric bounds (scalar limits)
 */
template<int N, typename T>
Math::Vec<N> saturateSymmetric(const Math::Vec<N>& value, T limit) {
    Math::Vec<N> result;
    for (int i = 0; i < N; ++i) {
        result(i) = saturate(value(i), -limit, limit);
    }
    return result;
}

#endif // CORE_SATURATE_HPP