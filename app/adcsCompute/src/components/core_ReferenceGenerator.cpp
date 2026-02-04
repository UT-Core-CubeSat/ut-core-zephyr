#include "core_ReferenceGenerator.hpp"
#include "../ADCSCore.hpp"  // For ADCS::Command, etc.

// ============================================================================
// CONSTRUCTOR
// ============================================================================
ReferenceGenerator::ReferenceGenerator() 
    : helpers_(),
      Ts_(Param::SimTime::Ts),
      last_q_ref_({1, 0, 0, 0}),
      initialized_(false),
      last_q_ff_({1, 0, 0, 0}),
      last_mode_(ADCS::MissionMode::SAFE),
      feedforward_initialized_(false)
{
}

// ============================================================================
// MAIN UPDATE
// ============================================================================
ReferenceGenerator::RefOutput ReferenceGenerator::update(
    const StateVector& states, 
    const ADCS::Command& command,
    Scalar unix_time) 
{
    RefGenOutput gen;
    Param::PointingMode internal_mode;

    // Dispatch based on commanded mission mode
    switch (command.mode) {
        case ADCS::MissionMode::SAFE:
            gen = handleSafe(states);
            internal_mode = Param::PointingMode::OFF;
            break;

        case ADCS::MissionMode::DETUMBLE:
            gen = handleDetumble(states);
            internal_mode = Param::PointingMode::DETUMBLE;
            break;

        case ADCS::MissionMode::STANDBY:
            gen = handleStandby(states, unix_time);
            internal_mode = Param::PointingMode::POINT;
            break;

        case ADCS::MissionMode::DOWNLINK:
            gen = handleDownlink(states, command.target, unix_time);
            internal_mode = Param::PointingMode::POINT;
            break;

        case ADCS::MissionMode::IMAGING:
            gen = handleImaging(states, command.target, command.track_target, unix_time);
            internal_mode = Param::PointingMode::POINT;
            break;

        case ADCS::MissionMode::CUSTOM:
            gen = handleCustom(states, 
                              Vector3{command.body_axis(0), command.body_axis(1), command.body_axis(2)},
                              Vector3{command.target_eci(0), command.target_eci(1), command.target_eci(2)},
                              unix_time);
            internal_mode = Param::PointingMode::POINT;
            break;

        default:
            gen = handleSafe(states);
            internal_mode = Param::PointingMode::OFF;
            break;
    }

    // Reset feedforward on mode change
    if (command.mode != last_mode_) {
        feedforward_initialized_ = false;
        last_mode_ = command.mode;
    }

    // Post-processing: enforce quaternion continuity
    gen.q_ref = enforceQuaternionContinuity(gen.q_ref);
    last_q_ref_ = gen.q_ref;

    // Pack output
    Reference out;
    out.setSegment(0, gen.q_ref);      // indices 0-3
    out.setSegment(4, gen.omega_ref);  // indices 4-6
    out.setSegment(7, gen.alpha_ref);  // indices 7-9

    return RefOutput{out, internal_mode};
}

// ============================================================================
// MODE HANDLERS
// ============================================================================

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleDetumble(const StateVector& states) {
    // DETUMBLE mode: Just hold current attitude, let BDot controller handle detumble
    RefGenOutput output;
    output.q_ref = states.segment<4>(6);  // Current attitude
    output.omega_ref = Vector3::Zero();    // Zero rate target
    output.alpha_ref = Vector3::Zero();
    return output;
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleSafe(const StateVector& states) {
    // SAFE mode: Just hold current attitude, reference won't matter
    RefGenOutput output;
    output.q_ref = states.segment<4>(6);  // Current attitude
    output.omega_ref = Vector3::Zero();    // Zero rate target
    output.alpha_ref = Vector3::Zero();
    return output;
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleStandby(
    const StateVector& states, 
    Scalar unix_time) 
{
    // STANDBY: Point solar panels (+Y) at Sun
    // Secondary: Keep star tracker (-Y) away from Sun (redundant with primary)
    //            Or: Point +X toward velocity for thermal balance
    
    PointingObjective primary;
    primary.body_axis = Param::Config::face_solar;  // +Y (solar panels)
    primary.target_eci = getSunVector(unix_time);
    primary.weight = 1.0;

    PointingObjective secondary;
    secondary.body_axis = Param::Config::face_boresight;  // -Z (camera/boresight)
    secondary.target_eci = getNadirVector(states);  // Point camera at Earth (useful)
    secondary.weight = 0.5;

    return computeAttitude(primary, secondary, states);
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleDownlink(
    const StateVector& states, 
    const ADCS::GroundTarget& target,
    Scalar unix_time) 
{
    // DOWNLINK: Point antenna (+X) at ground station
    // Secondary: Maximize solar power (+Y toward Sun)
    
    PointingObjective primary;
    primary.body_axis = Param::Config::face_antenna;  // +X (S-band antenna)
    primary.target_eci = getGroundTargetVector(states, target, unix_time);
    primary.weight = 1.0;

    PointingObjective secondary;
    secondary.body_axis = Param::Config::face_solar;  // +Y (solar panels)
    secondary.target_eci = getSunVector(unix_time);
    secondary.weight = 0.5;

    return computeAttitude(primary, secondary, states);
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleImaging(
    const StateVector& states,
    const ADCS::GroundTarget& target,
    bool track_target,
    Scalar unix_time)
{
    // IMAGING: Point boresight (-Z) at target or nadir
    // Secondary: Maximize solar power (+Y toward Sun)
    
    PointingObjective primary;
    primary.body_axis = Param::Config::face_boresight;  // -Z (camera)
    
    if (track_target) {
        primary.target_eci = getGroundTargetVector(states, target, unix_time);
    } else {
        primary.target_eci = getNadirVector(states);
    }
    primary.weight = 1.0;

    PointingObjective secondary;
    secondary.body_axis = Param::Config::face_solar;  // +Y (solar panels)
    secondary.target_eci = getSunVector(unix_time);
    secondary.weight = 0.5;

    return computeAttitude(primary, secondary, states);
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::handleCustom(
    const StateVector& states,
    const Vector3& body_axis,
    const Vector3& target_eci,
    Scalar unix_time)
{
    // CUSTOM: Direct control, still use Sun as secondary
    PointingObjective primary;
    primary.body_axis = body_axis.normalized();
    primary.target_eci = target_eci.normalized();
    primary.weight = 1.0;

    PointingObjective secondary;
    secondary.body_axis = Param::Config::face_solar;
    secondary.target_eci = getSunVector(unix_time);
    secondary.weight = 0.5;

    return computeAttitude(primary, secondary, states);
}

// ============================================================================
// CORE POINTING ALGORITHM
// ============================================================================

ReferenceGenerator::RefGenOutput ReferenceGenerator::computeAttitude(
    const PointingObjective& primary,
    const PointingObjective& secondary,
    const StateVector& states)
{
    // Use the two-vector pointing algorithm
    // Primary axis is satisfied exactly, secondary is best-effort
    
    Vector3 b1 = primary.body_axis.normalized();
    Vector3 t1 = primary.target_eci.normalized();
    Vector3 b2 = secondary.body_axis.normalized();
    Vector3 t2 = secondary.target_eci.normalized();

    RefGenOutput output;
    output.q_ref = helpers_.quatFromTwoVectorPairs(b1, t1, b2, t2);
    output.omega_ref = computeFeedforwardOmega(output.q_ref);
    output.alpha_ref = Vector3::Zero();
    
    return output;
}

// ============================================================================
// TARGET RESOLUTION
// ============================================================================

ReferenceGenerator::Vector3 ReferenceGenerator::getSunVector(Scalar unix_time) {
    Scalar jd = helpers_.julianDate(unix_time);
    return helpers_.earth2sun(jd).normalized();
}

ReferenceGenerator::Vector3 ReferenceGenerator::getNadirVector(const StateVector& states) {
    Vector3 R = states.segment<3>(0);  // Position in ECI
    return (-R).normalized();  // Nadir points toward Earth center
}

ReferenceGenerator::Vector3 ReferenceGenerator::getAntiSunVector(Scalar unix_time) {
    return -getSunVector(unix_time);
}

ReferenceGenerator::Vector3 ReferenceGenerator::getGroundTargetVector(
    const StateVector& states,
    const ADCS::GroundTarget& target,
    Scalar unix_time)
{
    // Convert ground target LLA to ECI
    // 1. LLA -> ECEF
    // 2. ECEF -> ECI (using current time)
    
    Scalar lat_rad = target.latitude * Shared::deg2rad;
    Scalar lon_rad = target.longitude * Shared::deg2rad;
    Scalar alt = target.altitude;
    
    // WGS84 ellipsoid parameters
    constexpr Scalar a = 6378137.0;           // Semi-major axis [m]
    constexpr Scalar f = 1.0 / 298.257223563; // Flattening
    constexpr Scalar e2 = 2*f - f*f;          // Eccentricity squared
    
    // Prime vertical radius of curvature
    Scalar sin_lat = std::sin(lat_rad);
    Scalar cos_lat = std::cos(lat_rad);
    Scalar N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    
    // ECEF position
    Vector3 r_ecef;
    r_ecef(0) = (N + alt) * cos_lat * std::cos(lon_rad);
    r_ecef(1) = (N + alt) * cos_lat * std::sin(lon_rad);
    r_ecef(2) = (N * (1.0 - e2) + alt) * sin_lat;
    
    // ECEF -> ECI rotation
    Scalar jd = helpers_.julianDate(unix_time);
    Param::Matrix3 R_ecef2eci = helpers_.dcmeci2ecef(jd).transpose();
    
    Vector3 r_target_eci = R_ecef2eci * r_ecef;
    
    // Vector from spacecraft to target
    Vector3 r_sc = states.segment<3>(0);  // Spacecraft position in ECI
    Vector3 los = r_target_eci - r_sc;    // Line of sight
    
    return los.normalized();
}

// ============================================================================
// UTILITIES
// ============================================================================

ReferenceGenerator::Quat ReferenceGenerator::enforceQuaternionContinuity(const Quat& q_new) {
    if (!initialized_) {
        initialized_ = true;
        return q_new;
    }
    if (last_q_ref_.dot(q_new) < 0) {
        return -q_new;
    }
    return q_new;
}

ReferenceGenerator::Vector3 ReferenceGenerator::computeFeedforwardOmega(const Quat& q_ref) {
    Vector3 omega_ref = Vector3::Zero();

    if (!feedforward_initialized_) {
        last_q_ff_ = q_ref;
        feedforward_initialized_ = true;
    } else {
        // Numerical differentiation: q_dot ≈ (q_new - q_old) / Ts
        Quat q_dot = (q_ref - last_q_ff_) / Ts_;
        
        // ω = 2 * q* ⊗ q_dot (vector part)
        Quat q_conj = helpers_.quatconj(last_q_ff_);
        Quat omega_quat = helpers_.quatMultiply(q_conj, q_dot);
        omega_ref = 2.0 * omega_quat.tail<3>();
        
        last_q_ff_ = q_ref;
    }
    
    return omega_ref;
}