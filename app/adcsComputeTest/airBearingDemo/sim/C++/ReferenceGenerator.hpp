#ifndef REFERENCE_GENERATOR_HPP
#define REFERENCE_GENERATOR_HPP

#include "Parameters.hpp"
#include "HelperFunctions.hpp"
#include <vector>

class ReferenceGenerator {
public: 
    // Type Aliases for readability 
    using Scalar = Param::Real;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;
    using StateVector = Param::Vector17;
    using Reference = Param::Vector10;

    // Constructor
    ReferenceGenerator();

    

    struct RefOutput {
        Reference reference;
        Param::PointingMode mode;
    };

    RefOutput update(const StateVector& states, const Scalar& t);

private: 

    struct RefGenOutput {
        Quat q_ref;
        Vector3 omega_ref;
        Vector3 alpha_ref;
    };

    std::vector<Param::MissionEvent> MissionSchedule;  // Plain vector

    // Private methods 
    RefGenOutput detumbleMode(const StateVector& states);
    RefGenOutput pointMode(const Param::MissionEvent& event, const StateVector& states, const Scalar& t);
    RefGenOutput slewMode(const StateVector& states, const Vector3& slew);
    RefGenOutput offMode(const StateVector& states);

    Param::MissionEvent check_schedule(const Scalar& t);
    Quat enforceQuaternionContinuity(const Quat& q_new);

    Vector3 resolveTargetVector(Param::TargetType type, const Vector3& vec, const StateVector& states, const Scalar& t);
    Vector3 computeFeedforwardOmega(const Quat& q_ref, Param::PointingMode current_mode);

    // Private Members 
    HelperFunctions helpers;
    Scalar Ts;
    Param::PointingMode mode;
    Param::Real DateTime;
    int currentIndex;
    Vector4 last_q_ref;
    bool initialized;

    // Feedforward state
    Quat last_q_ff;
    Param::PointingMode last_mode;
    int last_event_index;  // Track event transitions for feedforward reset
    bool feedforward_initialized;
};

#endif // REFERENCE_GENERATOR_HPP