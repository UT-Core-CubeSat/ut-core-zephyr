#ifndef CORE_REFERENCE_GENERATOR_HPP
#define CORE_REFERENCE_GENERATOR_HPP

#include "core_Parameters.hpp"
#include "core_HelperFunctions.hpp"

// Forward declare ADCS types to avoid circular includes
namespace ADCS {
    enum class MissionMode;
    struct GroundTarget;
    struct Command;
}

class ReferenceGenerator {
public: 
    using Scalar = Param::Real;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;
    using StateVector = Param::Vector17;
    using Reference = Param::Vector10;

    // Constructor
    ReferenceGenerator();

    // Output structure
    struct RefOutput {
        Reference reference;        // [q_ref(4), omega_ref(3), alpha_ref(3)]
        Param::PointingMode mode;   // Internal mode for controller
    };

    // Main update - now takes Command instead of time
    RefOutput update(const StateVector& states, 
                     const ADCS::Command& command, 
                     Scalar unix_time);

private: 
    // Internal reference output before packing
    struct RefGenOutput {
        Quat q_ref;
        Vector3 omega_ref;
        Vector3 alpha_ref;
    };

    // ========================================================================
    // MODE HANDLERS
    // ========================================================================
    RefGenOutput handleSafe(const StateVector& states);
    RefGenOutput handleDetumble(const StateVector& states);
    RefGenOutput handleStandby(const StateVector& states, Scalar unix_time);
    RefGenOutput handleDownlink(const StateVector& states, 
                                const ADCS::GroundTarget& target, 
                                Scalar unix_time);
    RefGenOutput handleImaging(const StateVector& states, 
                               const ADCS::GroundTarget& target,
                               bool track_target,
                               Scalar unix_time);
    RefGenOutput handleCustom(const StateVector& states,
                              const Vector3& body_axis,
                              const Vector3& target_eci,
                              Scalar unix_time);

    // ========================================================================
    // CORE POINTING ALGORITHM
    // ========================================================================
    // Computes fully-constrained attitude from primary and secondary objectives
    struct PointingObjective {
        Vector3 body_axis;      // Body frame axis to align
        Vector3 target_eci;     // Target direction in ECI frame
        Scalar weight;          // Priority weight (1.0 = primary, <1 = secondary)
    };
    
    RefGenOutput computeAttitude(const PointingObjective& primary,
                                 const PointingObjective& secondary,
                                 const StateVector& states);

    // ========================================================================
    // TARGET RESOLUTION
    // ========================================================================
    Vector3 getSunVector(Scalar unix_time);
    Vector3 getNadirVector(const StateVector& states);
    Vector3 getGroundTargetVector(const StateVector& states, 
                                  const ADCS::GroundTarget& target,
                                  Scalar unix_time);
    Vector3 getAntiSunVector(Scalar unix_time);  // For star tracker avoidance

    // ========================================================================
    // UTILITIES
    // ========================================================================
    Quat enforceQuaternionContinuity(const Quat& q_new);
    Vector3 computeFeedforwardOmega(const Quat& q_ref);

    // ========================================================================
    // MEMBERS
    // ========================================================================
    HelperFunctions helpers_;
    Scalar Ts_;
    
    // Continuity tracking
    Quat last_q_ref_;
    bool initialized_;
    
    // Feedforward state
    Quat last_q_ff_;
    ADCS::MissionMode last_mode_;
    bool feedforward_initialized_;
};

#endif // CORE_REFERENCE_GENERATOR_HPP