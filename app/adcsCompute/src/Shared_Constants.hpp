#ifndef SHARED_CONSTANTS_HPP
#define SHARED_CONSTANTS_HPP

// SHARED CONSTANTS
// Parameters that MUST be identical between Core (flight software) and Plant (sim)

namespace Shared {

    // Timing
    constexpr double Ts = 0.025;  // Sample period [s] - 40 Hz loop rate

    // Math constants
    constexpr double PI = 3.14159265358979323846;
    constexpr double deg2rad = PI / 180.0;
    constexpr double rad2deg = 180.0 / PI;
}

#endif // SHARED_CONSTANTS_HPP