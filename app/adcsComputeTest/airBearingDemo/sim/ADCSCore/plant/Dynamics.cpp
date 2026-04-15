#include "Dynamics.hpp" 
#include <iostream>
#include <cmath>

// Constructor 
Dynamics::Dynamics()
    : Ts(PlantParam::SimTime::Ts),
      I(PlantParam::Spacecraft::I),
      I_inv(Math::inverse3x3(PlantParam::Spacecraft::I)),
      states(PlantParam::Vector11::Zero()),
      reference(PlantParam::Vector10::Zero()),
      states_dot(PlantParam::Vector11::Zero()),
      S(PlantParam::Actuators::S),
      I_wheel(PlantParam::Actuators::I_wheel),
      h_cg(PlantParam::Apparatus::h_cg),
      m(PlantParam::Spacecraft::mass),
      g(PlantParam::Environment::g),
      r_dist(PlantParam::Apparatus::r_disturbance),
      b_friction(PlantParam::Apparatus::b_friction),
      t_current(static_cast<Scalar>(0.0)),
      i_motor(PlantParam::Vector4::Zero()), 
      ripple_phase(PlantParam::Vector4::Zero())
{
    states.setSegment(0, PlantParam::InitialState.q0);
    states.setSegment(4, PlantParam::InitialState.omega);
    states.setSegment(7, PlantParam::InitialState.omega_wheel);
}

void Dynamics::update(const InputVector& inputs) {
    // Update motor current dynamics (electrical model)
    if (PlantParam::Actuators::WheelDynamics::enable_motor_dynamics) {
        Vector4 tau_cmd = inputs.segment<4>(0);
        Vector4 omega_wheel = states.segment<4>(7);
        
        Scalar Kt = PlantParam::Actuators::WheelMotor::Kt;
        Scalar Ke = PlantParam::Actuators::WheelMotor::Ke;
        Scalar R = PlantParam::Actuators::WheelMotor::R;
        
        // Motor electrical time constant tau_e = L/R = 0.12mH / 3.95 ohm = 30 us
        // Since tau_e << Ts (sample period), electrical dynamics reach steady-state
        // nearly instantaneously relative to our sample rate. Use quasi-static model:
        // 
        // Steady-state current: i_ss = (V_cmd - Ke * omega) / R
        // With voltage command: V_cmd = R * i_desired + Ke * omega (back-EMF compensation)
        // We get: i_ss = i_desired = tau_cmd / Kt
        //
        // However, we model a first-order motor lag (mechanical time constant tau_m)
        // to capture the motor's mechanical response time (inertia + controller lag)
        Scalar tau_m = PlantParam::Actuators::WheelMotor::tau_m;
        
        // First-order lag on current: tau_m * di/dt + i = i_desired
        // Discrete update: i[k+1] = i[k] + (Ts/tau_m) * (i_desired - i[k])
        Scalar alpha = Ts / tau_m;
        if (alpha > static_cast<Scalar>(1.0)) {
            alpha = static_cast<Scalar>(1.0);  // Ensure stability
        }
        
        for (int i = 0; i < 4; ++i) {
            Scalar i_desired = tau_cmd(i) / Kt;
            i_motor(i) += alpha * (i_desired - i_motor(i));
        }
        
        // Clamp current to realistic limits (based on max torque)
        Scalar i_max = PlantParam::Actuators::tau_w_max / Kt;
        for (int i = 0; i < 4; ++i) {
            if (i_motor(i) > i_max) i_motor(i) = i_max;
            if (i_motor(i) < -i_max) i_motor(i) = -i_max;
        }
    }

    // Update ripple phase based on wheel spin rate
    if (PlantParam::Actuators::WheelDynamics::enable_ripple) {
        Vector4 omega_wheel = states.segment<4>(7);
        for (int i = 0; i < 4; ++i) {
            Scalar rate = PlantParam::Actuators::WheelDynamics::ripple_harmonic * std::abs(omega_wheel(i));
            ripple_phase(i) += rate * Ts;
            // Keep phase bounded
            if (ripple_phase(i) > static_cast<Scalar>(2.0) * PlantParam::PI) {
                ripple_phase(i) = std::fmod(ripple_phase(i), static_cast<Scalar>(2.0) * PlantParam::PI);
            }
        }
    }
    
    rk4_kinetics(inputs);
    t_current += Ts;
}

void Dynamics::rk4_kinetics(const InputVector& inputs) {
    PlantParam::Vector11 k1 = kinetics(states, inputs, t_current);
    PlantParam::Vector11 k2 = kinetics(states + (Ts / static_cast<Scalar>(2.0)) * k1, inputs, t_current + Ts/2);
    PlantParam::Vector11 k3 = kinetics(states + (Ts / static_cast<Scalar>(2.0)) * k2, inputs, t_current + Ts/2);
    PlantParam::Vector11 k4 = kinetics(states + Ts * k3, inputs, t_current + Ts);

    states = states + (Ts / static_cast<Scalar>(6.0)) * (k1 + static_cast<Scalar>(2.0) * k2 + static_cast<Scalar>(2.0) * k3 + k4);
    
    // Normalize quaternion
    PlantParam::Quat q_norm;
    q_norm(0) = states(0);
    q_norm.setSegment(1, states.segment<3>(1));
    q_norm.normalize();
    states.setSegment(0, q_norm);
    
    // Saturate wheel speeds
    states.setSegment(7, saturateSymmetric<4>(states.segment<4>(7), PlantParam::Actuators::omega_w_max));
}

PlantParam::Vector11 Dynamics::kinetics(const PlantParam::Vector11& states, 
                                         const InputVector& inputs,
                                         const Scalar& t) {
    // Unpack States
    Quat q = states.segment<4>(0);
    Vector3 omega = states.segment<3>(4);
    Vector4 omega_wheel = states.segment<4>(7);

    // Calculate forces and moments
    forceOutput fm = forces_and_moments(inputs, q, omega, t);
    Vector4 tau_rw = fm.moments.segment<4>(0);
    Vector3 m_cmd = fm.moments.segment<3>(4);
    // Get B-Field in body frame 
    Quat q_conj; q_conj(0) = q(0); q_conj.setSegment(1, -q.segment<3>(1));
    Vector3 B_body = helpers.quatRotate(q_conj, PlantParam::Apparatus::B_helmholtz);
    Vector3 tau_mtq = m_cmd.cross(B_body);  // Actual torque [Nm]
    Vector3 tau_gravity = fm.tau_gravity;
    Vector3 tau_disturbance = fm.tau_disturbance;
    Vector3 tau_friction = fm.tau_friction;

    // Reaction Wheel Angular Momentum
    Vector3 h_w = S * (I_wheel * omega_wheel);
    
    // Total external torque (actuators + environment)
    Vector3 tau_ext = tau_mtq - (S * tau_rw) + tau_gravity + tau_disturbance + tau_friction;

    // Quaternion kinematics
    Vector3 q_vec = q.segment<3>(1);
    Scalar q0 = q(0);
    Vector3 q_dot_vec = static_cast<Scalar>(0.5) * (q0 * omega + q_vec.cross(omega));
    Scalar q0_dot = static_cast<Scalar>(-0.5) * q_vec.dot(omega);

    Quat q_dot; 
    q_dot(0) = q0_dot;
    q_dot.setSegment(1, q_dot_vec);
    
    // Euler's equation: I*ω̇ = τ_ext - ω × (I*ω + h_w)
    Vector3 omega_dot = I_inv * (tau_ext - omega.cross(I * omega + h_w));

    // Wheel dynamics
    Vector4 omega_wheel_dot = tau_rw / I_wheel;

    // Pack output
    PlantParam::Vector11 states_dot_local;
    states_dot_local.setSegment(0, q_dot);
    states_dot_local.setSegment(4, omega_dot);
    states_dot_local.setSegment(7, omega_wheel_dot);
    
    // Store for sensor access
    states_dot = states_dot_local;
    
    return states_dot_local;
}

Dynamics::forceOutput Dynamics::forces_and_moments(const InputVector& inputs,
                                                    const Quat& q,
                                                    const Vector3& omega,
                                                    const Scalar& t) {
    
    Vector3 tau_mtq = inputs.segment<3>(4);
    
    Vector4 omega_wheel = states.segment<4>(7);
    // =====================================================================
    // 1. GRAVITY TORQUE
    // =====================================================================
    // CG offset vector in body frame: r_cg = [0, 0, h_cg]
    // (assuming CG is displaced along body Z-axis from pivot)
    Vector3 r_cg_body = Vector3{static_cast<Scalar>(0.0), 
                                 static_cast<Scalar>(0.0), 
                                 h_cg};
    
    // Gravity vector in inertial frame: g_i = [0, 0, -g]
    Vector3 g_inertial = PlantParam::Environment::g_inertial;
    
    // Rotate gravity to body frame: g_b = C_bi * g_i
    // Using quaternion: g_b = q* ⊗ g_i ⊗ q (conjugate rotation)
    Quat q_conj;
    q_conj(0) = q(0);
    q_conj.setSegment(1, -q.segment<3>(1));
    Vector3 g_body = helpers.quatRotate(q_conj, g_inertial);
    
    // Gravity torque: τ_g = r_cg × (m * g_b)
    Vector3 tau_gravity = r_cg_body.cross(m * g_body);
    
    // =====================================================================
    // 2. DISTURBANCE TORQUE (Finger Push)
    // =====================================================================
    Vector3 tau_disturbance = Vector3::Zero();
    
    // Periodic disturbance: active during [kT, kT + T_dur] for k = 0,1,2,...
    Scalar T_period = PlantParam::Apparatus::T_disturbance_period;
    Scalar T_dur = PlantParam::Apparatus::T_disturbance_duration;
    Scalar t_mod = std::fmod(t, T_period);
    
    if (t_mod < T_dur && t > static_cast<Scalar>(1.0)) {  // Skip first second for settling
        // Disturbance force in body frame
        Vector3 F_dist = PlantParam::Apparatus::F_disturbance_max * 
                         PlantParam::Apparatus::disturbance_direction;
        
        // Application point (at edge of cubesat, perpendicular to force)
        // Assuming force applied at radius r_dist from pivot
        Vector3 r_dist_body = Vector3{static_cast<Scalar>(0.0), 
                                       static_cast<Scalar>(0.0), 
                                       r_dist};
        
        // Torque: τ = r × F
        tau_disturbance = r_dist_body.cross(F_dist);
    }

    // Wheel internal dynamics: friction and torque ripple
    Vector4 tau_fric = Vector4::Zero();
    Vector4 tau_ripple = Vector4::Zero();

    if (PlantParam::Actuators::WheelDynamics::enable_friction) {
        tau_fric = compute_wheel_friction(omega_wheel);
    }

    if (PlantParam::Actuators::WheelDynamics::enable_ripple) {
        tau_ripple = compute_wheel_ripple(omega_wheel);
    }

    // Compute motor torque from current: tau_motor = Kt * i
    Vector4 tau_motor = Vector4::Zero();
    if (PlantParam::Actuators::WheelDynamics::enable_motor_dynamics) {
        Scalar Kt = PlantParam::Actuators::WheelMotor::Kt;
        tau_motor = Kt * i_motor;
    } else {
        // If motor dynamics disabled, use commanded torque directly
        tau_motor = inputs.segment<4>(0);
        tau_motor = saturateSymmetric<4>(tau_motor, PlantParam::Actuators::tau_w_max);
    }

    // Net wheel torque applied to wheel (motor + ripple - friction)
    Vector4 tau_wheel_net = tau_motor + tau_ripple - tau_fric;
    
    // =====================================================================
    // 3. AIR BEARING FRICTION
    // =====================================================================
    // Viscous friction: τ_f = -b * ω
    Vector3 tau_friction = -b_friction * omega;
    
    // =====================================================================
    // 4. ACTUATOR MOMENTS (passthrough)
    // =====================================================================
    InputVector moments;
    moments.setSegment(0, tau_wheel_net);

    // Store for logging
    tau_gravity_current = tau_gravity;
    tau_disturbance_current = tau_disturbance;

    return forceOutput{Vector3::Zero(), moments, tau_gravity, tau_disturbance, tau_friction};
}

Dynamics::Vector4 Dynamics::compute_motor_current_dynamics(const Vector4& V_cmd, const Vector4& omega_wheel, const Vector4& i_motor) {
    // Electrical motor current dynamics
    // di/dt = (V - Ke*omega - R*i) / L
    // where:
    //   V = applied voltage (derived from torque command)
    //   Ke = back-EMF constant [V/(rad/s)]
    //   R = phase resistance [ohm]
    //   L = phase inductance [H]
    
    using Real = Scalar;
    
    Real Ke = PlantParam::Actuators::WheelMotor::Ke;
    Real R = PlantParam::Actuators::WheelMotor::R;
    Real L = PlantParam::Actuators::WheelMotor::L;
    
    Vector4 di_dt = Vector4::Zero();
    
    for (int i = 0; i < 4; ++i) {
        Real V = V_cmd(i);
        Real omega = omega_wheel(i);
        Real current = i_motor(i);
        
        // Voltage across motor = V_applied - back_EMF - resistive_drop
        Real V_net = V - Ke * omega - R * current;
        
        // di/dt = V_net / L
        di_dt(i) = V_net / L;
    }
    
    return di_dt;
}

// ============================================================================
// WHEEL FRICTION MODEL
// ============================================================================
// Viscous friction: τ_f = -b_w * ω_w
// where b_w is the friction coefficient [Nm·s/rad]
Dynamics::Vector4 Dynamics::compute_wheel_friction(const Vector4& omega_wheel) const {
    Vector4 tau_fric = Vector4::Zero();
    Scalar b = PlantParam::Actuators::WheelDynamics::b_viscous;
    Scalar tau_c = PlantParam::Actuators::WheelDynamics::tau_coulomb;
    Scalar omega_eps = PlantParam::Actuators::WheelDynamics::omega_eps;

    for (int i = 0; i < 4; ++i) {
        Scalar sign = std::tanh(omega_wheel(i) / omega_eps);
        tau_fric(i) = b * omega_wheel(i) + tau_c * sign;
    }

    return tau_fric;
}

// ============================================================================
// WHEEL TORQUE RIPPLE MODEL
// ============================================================================
// Ripple torque: τ_ripple = A_ripple * sin(harmonic * phase)
// Models cogging torque and other periodic effects
Dynamics::Vector4 Dynamics::compute_wheel_ripple(const Vector4& omega_wheel) const {
    Vector4 tau_ripple = Vector4::Zero();
    Scalar A_ripple = PlantParam::Actuators::WheelDynamics::ripple_amp;
    Scalar harmonic = PlantParam::Actuators::WheelDynamics::ripple_harmonic;
    
    for (int i = 0; i < 4; ++i) {
        // Only add ripple if wheel is spinning
        if (std::abs(omega_wheel(i)) > static_cast<Scalar>(0.1)) {
            tau_ripple(i) = A_ripple * std::sin(harmonic * ripple_phase(i));
        }
    }
    
    return tau_ripple;
}