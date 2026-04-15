#include "ReferenceGenerator.hpp"
#include "MissionSchedule.hpp"
#include <iostream> // For debugging purposes


// Constructor 
ReferenceGenerator::ReferenceGenerator() 
    : // Initialize Members 
    Ts(Param::SimTime::Ts),
    mode(Param::PointingMode::OFF),
    DateTime(Param::SimTime::epoch_timestamp),
    currentIndex(0),
    last_q_ref(Vector4::Zero()),
    initialized(false),
    
    // Feedforward state
    last_q_ff(Vector4::Zero()),
    last_mode(Param::PointingMode::OFF),
    last_event_index(-1),  // Initialize to invalid so first event triggers reset
    feedforward_initialized(false),
    MissionSchedule(Mission::getSchedule())
{
    last_q_ff(0) = 1.0; // Initialize to identity quaternion
}

ReferenceGenerator::RefOutput ReferenceGenerator::update(const StateVector& states, const Scalar& t) {
    // 1. Get current mission event
    Param::MissionEvent event = check_schedule(t);
    RefGenOutput gen;

    // 2. Main Mode Switch
    switch (event.mode) {
        case Param::PointingMode::DETUMBLE:
            gen = detumbleMode(states);
            break;

        case Param::PointingMode::POINT:
            gen = pointMode(event, states, t);
            break;

        case Param::PointingMode::OFF:
        default:
            gen = offMode(states);
            break;
    }

    // 3. Post-Processing: Enforce Continuity 
    // This prevents the quaternion from flipping 180 degrees between steps
    gen.q_ref = enforceQuaternionContinuity(gen.q_ref); 
    last_q_ref = gen.q_ref;

    // 4. Pack and Return
    Param::Vector10 out;
    out.setSegment(0, gen.q_ref);         // 0-3
    out.setSegment(4, gen.omega_ref);
    out.setSegment(7, gen.alpha_ref);     // 7-9
    return RefOutput{out, event.mode};
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::detumbleMode(const StateVector& states) {
    // Placeholder implementation for detumble mode 
    RefGenOutput output;
    output.q_ref = states.segment<4>(6);
    output.omega_ref = Vector3::Zero();
    output.alpha_ref = Vector3::Zero();
    return output;
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::pointMode(
    const Param::MissionEvent& event, 
    const StateVector& states, 
    const Scalar& t) 
{
    // Resolve primary target vector
    Vector3 t1 = resolveTargetVector(event.target, event.targetVec, states, t);
    
    // Resolve secondary target vector
    Vector3 t2 = resolveTargetVector(event.secondTarget, event.targetVec2, states, t);
    
    // Get body axes
    Vector3 b1 = event.face.normalized();
    Vector3 b2 = event.secondFace.normalized();
    
    // Compute fully-constrained quaternion
    RefGenOutput output;
    output.q_ref = helpers.quatFromTwoVectorPairs(b1, t1, b2, t2);
    
    // Compute feedforward angular velocity from quaternion rate
    output.omega_ref = computeFeedforwardOmega(output.q_ref, Param::PointingMode::POINT);

    output.alpha_ref = Vector3::Zero();
    
    return output;
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::slewMode(const StateVector& states, const Vector3& slew) {
    RefGenOutput output;
    output.q_ref = states.segment<4>(6);
    output.omega_ref = slew;
    output.alpha_ref = Vector3::Zero();
    return output;
}

ReferenceGenerator::RefGenOutput ReferenceGenerator::offMode(const StateVector& states) {
    RefGenOutput output;
    output.q_ref = states.segment<4>(6);
    output.omega_ref = Vector3::Zero();
    output.alpha_ref = Vector3::Zero();
    return output;
}

Param::MissionEvent ReferenceGenerator::check_schedule(const Scalar& t) {
    Param::MissionEvent event = MissionSchedule[currentIndex];
    
    if (t >= event.t_end && currentIndex < (int)MissionSchedule.size() - 1) {
        currentIndex++;
        event = MissionSchedule[currentIndex];
    }
    return event;
}

ReferenceGenerator::Quat ReferenceGenerator::enforceQuaternionContinuity(const Quat& q_new) {
    if (!initialized) {
        initialized = true;
        return q_new;
    }
    // Ensure continuity 
    if (last_q_ref.dot(q_new) < 0) {
        return -q_new;
    }
    return q_new;
}

ReferenceGenerator::Vector3 ReferenceGenerator::resolveTargetVector(
    Param::TargetType type, 
    const Vector3& vec, 
    const StateVector& states, 
    const Scalar& t) 
{
    switch (type) {
        case Param::TargetType::SUN: {
            Scalar current_unix_time = static_cast<Scalar>(DateTime) + t;
            Scalar jd = helpers.julianDate(current_unix_time);
            return helpers.earth2sun(jd).normalized();
        }
        
        case Param::TargetType::NADIR: {
            Vector3 R = states.segment<3>(0);
            return (-R).normalized();
        }
        
        case Param::TargetType::VECTOR:
        default:
            return vec.normalized();
    }
}

ReferenceGenerator::Vector3 ReferenceGenerator::computeFeedforwardOmega(
    const Quat& q_ref, 
    Param::PointingMode current_mode) 
{
    Vector3 omega_ref = Vector3::Zero();
    
    // Reset on mode transition OR event transition (e.g., switching between different POINT targets)
    if (current_mode != last_mode || currentIndex != last_event_index) {
        feedforward_initialized = false;
        last_mode = current_mode;
        last_event_index = currentIndex;
    }
    
    if (!feedforward_initialized) {
        // First call after mode change - no derivative available
        last_q_ff = q_ref;
        feedforward_initialized = true;
    } else {
        // Numerical differentiation of quaternion
        // q_dot ≈ (q_new - q_old) / Ts
        Quat q_dot = (q_ref - last_q_ff) / Ts;
        
        // Angular velocity from quaternion derivative:
        // ω = 2 * q_conj * q_dot  (vector part only)
        // 
        // Using: q_dot = 0.5 * q ⊗ [0; ω]
        // Rearranging: [0; ω] = 2 * q* ⊗ q_dot
        
        Quat q_conj = helpers.quatconj(last_q_ff);
        Quat omega_quat = helpers.quatMultiply(q_conj, q_dot);
        
        // Extract vector part and scale by 2
        omega_ref = 2.0 * omega_quat.tail<3>();
        
        // Update stored quaternion
        last_q_ff = q_ref;
    }
    
    return omega_ref;
}