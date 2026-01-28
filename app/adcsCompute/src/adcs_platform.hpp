#pragma once

#include "ADCSCore.hpp"

namespace ADCSPlatform {

// Time: for now, seconds since boot (stored in unix_time field)
double getUnixTime();

// Sensor getters (stubbed)
Math::Vec<3> getGyroData();
Math::Vec<4> getStarTrackerQuaternion();     // scalar-first [q0,q1,q2,q3]
Math::Vec<6> getCoarseSunSensorCurrents();   // 6 faces
Math::Vec<3> getMagnetometerData();
Math::Vec<6> getGPSECEFData();               // [x y z vx vy vz] ECEF
Math::Vec<4> getReactionWheelSpeeds();

// Actuator senders (stubbed)
void sendWheelTorques(const Math::Vec<4>& wheel_torque);
void sendMTQDipoles(const Math::Vec<3>& mtq_dipole);

} // namespace ADCSPlatform
