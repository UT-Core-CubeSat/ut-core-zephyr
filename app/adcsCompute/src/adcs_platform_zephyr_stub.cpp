#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "adcs_platform.hpp"

namespace {

// Small helper so you can printk any Vec<N> without iostream.
template<int N>
void printkVec(const char* label, const Math::Vec<N>& v) {
    printk("%s[", label);
    for (int i = 0; i < N; ++i) {
        printk("%f", v(i));
        if (i < N - 1) printk(", ");
    }
    printk("]\n");
}

} // namespace

namespace ADCSPlatform {

double getUnixTime() {
    // True UNIX time comes from GPS/RTC/CDH. Today: seconds since boot.
    return static_cast<double>(k_uptime_get()) * 1e-3;
}

Math::Vec<3> getGyroData() {
    // [rad/s] body rates
    return Math::Vec<3>{0.0, 0.0, 0.0};
}

Math::Vec<4> getStarTrackerQuaternion() {
    // scalar-first quaternion [q0,q1,q2,q3] = identity
    return Math::Vec<4>{1.0, 0.0, 0.0, 0.0};
}

Math::Vec<6> getCoarseSunSensorCurrents() {
    // [A] currents for 6 faces (arbitrary, but non-zero)
    return Math::Vec<6>{0.8, 0.2, 0.05, 0.0, 0.0, 0.1};
}

Math::Vec<3> getMagnetometerData() {
    // [T] rough Earth field magnitude scale
    return Math::Vec<3>{25e-6, -5e-6, 40e-6};
}

Math::Vec<6> getGPSECEFData() {
    // [m, m/s] x y z vx vy vz (totally fake)
    return Math::Vec<6>{1.0e6, 2.0e6, 3.0e6, 0.0, 7.5e3, 0.0};
}

Math::Vec<4> getReactionWheelSpeeds() {
    // [rad/s] 4 wheels
    return Math::Vec<4>{0.0, 0.0, 0.0, 0.0};
}

void sendWheelTorques(const Math::Vec<4>& t) {
    printkVec<4>("RW torque cmd: ", t);
}

void sendMTQDipoles(const Math::Vec<3>& m) {
    printkVec<3>("MTQ dipole cmd: ", m);
}

} // namespace ADCSPlatform
