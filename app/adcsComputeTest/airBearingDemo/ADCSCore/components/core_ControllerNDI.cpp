#include "core_ControllerNDI.hpp"
#include <iostream>  // DEBUG

// Constructor 
ControllerNDI::ControllerNDI()
    : // Initialize Members — order matches declaration in core_ControllerNDI.hpp
    I(Param::Spacecraft::I),
    S(Param::Actuators::S),
    S_pseudo(Param::Actuators::S_pseudo),
    N(Param::Actuators::N),
    I_wheel(Param::Actuators::I_wheel),
    omega_w_max(Param::Actuators::omega_w_max),
    omega_w_min(Param::Actuators::omega_w_min),
    tau_w_max(Param::Actuators::tau_w_max),
    tau_w_min(Param::Actuators::tau_w_min),
    lambda_min_model(Param::Controller::lambda_min_model),
    x_m(StateVector::Zero()),
    k_desat(Param::Actuators::k_desat),
    k_null(Param::Actuators::k_null),
    k_deadband(Param::Actuators::k_deadband),
    m_max(Param::Actuators::m_max),
    m_min(Param::Actuators::m_min),
    omega_deadband(Param::Actuators::omega_deadband),
    tau_rev_min_frac(Param::Actuators::tau_rev_min_frac),
    h_cg(Param::Apparatus::h_cg),
    m(Param::Spacecraft::mass),
    is_saturated(false),
    desat_active(false),
    desat_entry_timer(static_cast<Param::Real>(0.0)),
    desat_exit_timer(static_cast<Param::Real>(0.0)),
    desat_momentum_target(Vector3::Zero())
{
    // Calculate gains 
    calculate_gains(Param::Controller::t_s_model,
                    Param::Controller::zeta_model,
                    Param::Controller::t_s_plant,
                    Param::Controller::zeta_plant);

    x_m(0) = 0; x_m(1) = 0; x_m(2) = 0; x_m(3) = 1; // identity quaternion
    x_m(4) = 0; x_m(5) = 0; x_m(6) = 0; // zero body rates

}

ControllerNDI::NDIOutput ControllerNDI::update(const Param::Vector11& states,
                                     const Param::Vector10& reference,
                                     const Param::Vector13& measurements,
                                     Param::Real dt,
                                     bool enable_desat,
                                     bool apply_deadband)
{
    // Reference
    Quat q_r; 
    q_r(0) = reference(1);
    q_r(1) = reference(2);
    q_r(2) = reference(3);
    q_r(3) = reference(0);
    q_r.normalize();

    Vector3 omega_r = reference.segment<3>(4);
    Vector3 alpha_r = reference.segment<3>(7);
    Reference NDI_reference;
    NDI_reference.setSegment(0, q_r);
    NDI_reference.setSegment(4, omega_r);
    NDI_reference.setSegment(7, alpha_r);

    // States
    Quat q;
    q(0) = states(1);
    q(1) = states(2);
    q(2) = states(3);
    q(3) = states(0);
    q.normalize();
    Vector3 omega = states.segment<3>(4);
    Vector4 omega_w = states.segment<4>(7);

    // Outer Loop (Model -> Reference)
    StateVector x_model = update_reference_model(NDI_reference, dt);
    Quat q_m = x_model.segment<4>(0);
    Vector3 omega_m = x_model.segment<3>(4);

    StateVector x_model_dot = reference_model_dif_eq(x_model, NDI_reference);
    Vector3 omega_m_dot = x_model_dot.segment<3>(4);
    Param::Vector10 RegInput; 
    RegInput.setSegment(0, q_m);
    RegInput.setSegment(4, omega_m);
    RegInput.setSegment(7, omega_m_dot);

    // Regularize Reference 
    RegOutput reg_out = regularize_reference(q, RegInput);
    q_m = reg_out.q_ref_eff;
    omega_m = reg_out.omega_ref_eff;
    omega_m_dot = reg_out.alpha_ref_eff;
    StateVector toolIn;
    StateVector toolIn2; 
    toolIn.setSegment(0, q);
    toolIn.setSegment(4, omega);
    toolIn2.setSegment(0, q_m);
    toolIn2.setSegment(4, omega_m);

    // Inner Loop (Plant -> Model)
    ToolBoxOutput toolbox = compute_BP_Toolbox(toolIn, toolIn2, false);
    Param::Matrix43 E = toolbox.E_m;
    Param::Matrix3 C_BB = toolbox.C_BB;
    Scalar a = toolbox.a;

    // Compute Errors 
    Vector3 e_q = 2*E.transpose() * q_m;
    Vector3 e_omega = omega - C_BB * omega_m;
    Vector3 e_omega_dot = a*e_q - a1_plant*e_omega;
    Vector3 crossComp = C_BB*omega_m;
    Vector3 omega_dot = C_BB*omega_m_dot + crossComp.cross(omega) + e_omega_dot;

    // Wheel angular momentum in body frame — needed for Euler equation and desaturation
    Vector3 h_w = S * (I_wheel * omega_w);

    // Full spacecraft Euler equation with gyroscopic wheel coupling:
    // τ_ext = I·ω̇ + ω × (I·ω + h_w)
    // The h_w term is the gyroscopic cross-coupling from spinning reaction wheels.
    // Omitting it leaves an unmodelled disturbance that grows with wheel speed.
    Vector3 tau_NDI = I*omega_dot + omega.cross(I*omega + h_w);

    // Gravity feedforward Compensation
    Vector3 accel_meas = measurements.segment<3>(0);
    Scalar accel_norm = accel_meas.norm();
    if (accel_norm > static_cast<Scalar>(0.1)) {
        // Gravity direction in body frame
        Vector3 g_body = -accel_meas * (Param::g / accel_norm);

        // CG offset vector in body frame: r_cg = [0, 0, h_cg]
        Vector3 r_cg = Vector3{static_cast<Scalar>(0.0),
                               static_cast<Scalar>(0.0),
                               h_cg};

        // Gravity torque to cancel: τ_g = r_cg × (m * g_body)
        Vector3 tau_gravity_ff = r_cg.cross(m * g_body);

        // SUBTRACT to cancel gravity torque (plant adds +τ_gravity, so we need -τ_gravity)
        tau_NDI = tau_NDI - tau_gravity_ff;
    }

    Vector3 B_meas = measurements.segment<3>(6);

    Scalar mtq_comp_scale = static_cast<Scalar>(1.0);
    Scalar attitude_scale = static_cast<Scalar>(1.0);
    Scalar null_scale = static_cast<Scalar>(1.0);
    DesatOutput desat_out{Vector3::Zero(), Vector3::Zero()};

    if (enable_desat) {
        // Desaturation state machine (BOTH mode only):
        // Enter based on wheel saturation/high momentum (with mild rate guard),
        // then unload slowly while reducing wheel-vs-MTQ fighting.
        Scalar body_rate_norm = omega.norm();
        Scalar wheel_momentum_norm = h_w.norm();

        Scalar max_wheel_speed = static_cast<Scalar>(0.0);
        for (int i = 0; i < 4; ++i) {
            Scalar abs_speed = std::abs(omega_w(i));
            if (abs_speed > max_wheel_speed) max_wheel_speed = abs_speed;
        }

        bool rate_ready = body_rate_norm <= Param::Controller::Desat::entry_rate_norm;
        bool enter_by_speed = max_wheel_speed >= (Param::Controller::Desat::enter_wheel_speed_ratio * omega_w_max);
        bool enter_by_momentum = wheel_momentum_norm >= Param::Controller::Desat::enter_momentum_norm;
        bool desat_enter_condition = rate_ready && (enter_by_speed || enter_by_momentum);

        bool exit_by_speed = max_wheel_speed <= (Param::Controller::Desat::exit_wheel_speed_ratio * omega_w_max);
        bool exit_by_momentum = wheel_momentum_norm <= Param::Controller::Desat::exit_momentum_norm;
        bool desat_exit_condition = exit_by_speed && exit_by_momentum;

        if (desat_active) {
            if (desat_exit_condition) {
                desat_exit_timer += dt;
                if (desat_exit_timer >= Param::Controller::Desat::exit_hold_time) {
                    desat_active = false;
                    desat_entry_timer = static_cast<Param::Real>(0.0);
                    desat_exit_timer = static_cast<Param::Real>(0.0);
                    desat_momentum_target = Vector3::Zero();
                }
            } else {
                desat_exit_timer = static_cast<Param::Real>(0.0);
            }
        } else {
            if (desat_enter_condition) {
                desat_entry_timer += dt;
                if (desat_entry_timer >= Param::Controller::Desat::enter_hold_time) {
                    desat_active = true;
                    desat_entry_timer = static_cast<Param::Real>(0.0);
                    desat_exit_timer = static_cast<Param::Real>(0.0);
                    desat_momentum_target = h_w;
                }
            } else {
                desat_entry_timer = static_cast<Param::Real>(0.0);
            }
        }

        if (desat_active) {
            Scalar alpha = dt / (Param::Controller::Desat::filter_tau + dt);
            desat_momentum_target = (static_cast<Param::Real>(1.0) - alpha) * desat_momentum_target + alpha * h_w;

            desat_out = wheel_desaturate(B_meas, desat_momentum_target);
            desat_out.m_cmd *= Param::Controller::Desat::mtq_dipole_scale;
            desat_out.m_cmd = saturateSymmetric(desat_out.m_cmd, m_max);
            desat_out.tau_mtq_expected = desat_out.m_cmd.cross(B_meas);

            mtq_comp_scale = Param::Controller::Desat::wheel_mtq_comp_scale;
            attitude_scale = Param::Controller::Desat::wheel_attitude_scale;
            null_scale = static_cast<Scalar>(0.0);
        }
    } else {
        // MOTOR mode: no MTQ, reset desat state so it's clean if BOTH is entered later
        desat_active = false;
        desat_entry_timer = static_cast<Param::Real>(0.0);
        desat_exit_timer = static_cast<Param::Real>(0.0);
        desat_momentum_target = Vector3::Zero();
        mtq_comp_scale = static_cast<Scalar>(0.0);
    }

    Vector3 tau_mtq = desat_out.tau_mtq_expected;

    // Allocate to wheels (includes null-space equalization and deadband enforcement)
    Vector4 tau_tilde = allocateActuators(tau_NDI, tau_mtq, omega_w,
                                          mtq_comp_scale,
                                          attitude_scale,
                                          null_scale,
                                          apply_deadband);

    // Anti windup: check saturation for next timestep
    is_saturated = checkWheelSaturation(tau_tilde, omega_w);
    // Apply torque and speed saturation (includes back-EMF reversal protection)
    Vector4 tau_wheel = applyWheelSaturation(tau_tilde, omega_w);

    // Convert wheel torques → desired RPM command for downstream motor controller.
    // Integrate one timestep of wheel dynamics: omega_w_next = omega_w + (tau/I_wheel)*dt
    Vector4 omega_w_cmd = omega_w + (tau_wheel / I_wheel) * dt;
    for (int i = 0; i < 4; ++i) {
        omega_w_cmd(i) = saturate(omega_w_cmd(i), omega_w_min, omega_w_max);
    }
    Vector4 rpm_cmd = omega_w_cmd * (static_cast<Scalar>(60.0) / (static_cast<Scalar>(2.0) * Param::PI));

    return NDIOutput{rpm_cmd, desat_out.m_cmd};
}

ControllerNDI::StateVector ControllerNDI::reference_model_dif_eq(const StateVector& x_m, const Reference& reference) 
{
    // Reference model based off linear error dynamics from B&P Paper.
    Quat q_ref = reference.segment<4>(0);
    Vector3 omega_ref = reference.segment<3>(4);
    Vector3 alpha_ref = reference.segment<3>(7);
    Quat q_m = x_m.segment<4>(0);
    q_m = q_m / q_m.norm();
    Vector3 omega_m = x_m.segment<3>(4);
    Reference ref; 
    ref.setSegment(0, q_ref);
    ref.setSegment(4, omega_ref);
    ref.setSegment(7, alpha_ref);
    RegOutput reg_out = regularize_reference(q_m, ref);
    q_ref = reg_out.q_ref_eff;
    omega_ref = reg_out.omega_ref_eff;
    alpha_ref = reg_out.alpha_ref_eff;
    StateVector x_curr; 
    x_curr.setSegment(0, q_m);
    x_curr.setSegment(4, omega_m);
    StateVector x_ref; 
    x_ref.setSegment(0, q_ref);
    x_ref.setSegment(4, omega_ref);
    ToolBoxOutput toolbox = compute_BP_Toolbox(x_curr, x_ref, true);

    Param::Matrix43 E_m = toolbox.E_m;
    Param::Matrix3 C_BB_m = toolbox.C_BB;
    Scalar a_m = toolbox.a;

    Vector3 e_q = 2*E_m.transpose() * q_ref;
    Vector3 e_omega = omega_m - C_BB_m * omega_ref;
    Vector3 e_omega_dot = a_m*e_q - a1_model*e_omega;
    Quat x_m_dot_q = (E_m*omega_m)/2;
    Vector3 crosscomp = C_BB_m*omega_ref;
    Vector3 x_m_dot_omega = C_BB_m*alpha_ref + crosscomp.cross(omega_m) + e_omega_dot;
    StateVector res; 
    res.setSegment(0, x_m_dot_q);
    res.setSegment(4, x_m_dot_omega);
    return res;
}

ControllerNDI::StateVector ControllerNDI::update_reference_model(const Reference& reference, Param::Real dt) 
{
    // Integrate ODE using Heun's method (RK2) for better performance
    StateVector k1 = reference_model_dif_eq(x_m, reference);
    StateVector k2 = reference_model_dif_eq(x_m + dt * k1, reference);
    if (is_saturated) {
        x_m.setSegment(0, x_m.segment<4>(0) + (dt/2)*(k1.segment<4>(0) + k2.segment<4>(0)));
        x_m.setSegment(0, x_m.segment<4>(0).normalized());
        return x_m;
    }
    x_m += (dt / 2) * (k1 + k2);
    x_m.setSegment(0, x_m.segment<4>(0).normalized()); // Normalize Quaternion
    return x_m;
}

Param::Matrix43 ControllerNDI::compute_E(const Quat& q) {
    Param::Matrix34 E;
    Scalar q1 = q(0), q2 = q(1), q3 = q(2), q4 = q(3);
    E(0,0) = q4;  E(0,1) = q3;  E(0,2) = -q2; E(0,3) = -q1;
    E(1,0) = -q3; E(1,1) = q4;  E(1,2) = q1;  E(1,3) = -q2;
    E(2,0) = q2;  E(2,1) = -q1; E(2,2) = q4;  E(2,3) = -q3;
    return E.transpose();
}

void ControllerNDI::calculate_gains(const Scalar& t_s_model, 
                                  const Scalar& zeta_model, 
                                  const Scalar& t_s_plant, 
                                  const Scalar& zeta_plant) 
{
    // EQ's based off of Settling time, rather than rise time. 
    // Nat Frequencies 

    Scalar omega_n_model = 4/(zeta_model*t_s_model);
    Scalar omega_n_plant = 4/(zeta_plant*t_s_plant);

    // Error Gains
    a1_model = 2*zeta_model*omega_n_model;
    a0_model = omega_n_model * omega_n_model;
    // Plant Gains
    a1_plant = 2*zeta_plant*omega_n_plant;
    a0_plant = omega_n_plant * omega_n_plant;
}

ControllerNDI::ToolBoxOutput ControllerNDI::compute_BP_Toolbox(const StateVector& states, 
                                                              const StateVector& states_desired,
                                                              bool is_outer_loop) 
{
    // B&P ToolBox. Creates linearized error dynamics from
    // unique properties of quaternion kinematics. 

    // This "toolbox" will compute all quantities necessary to linearly 
    // drive states to states_desired.

    // Unpack states 
    Quat q = states.segment<4>(0);
    // q = q / q.norm();  // REDUNDANT - callers already normalize
    Vector3 omega = states.segment<3>(4);
    Quat q_desired = states_desired.segment<4>(0);
    // q_desired = q_desired / q_desired.norm();  // REDUNDANT - callers already normalize
    Vector3 omega_desired = states_desired.segment<3>(4);

    Scalar lambda = q.dot(q_desired);

    // Clamp lambda to prevent division by near-zero 
    // Safety net for edge cases where the model diverges enough from the plant 
    constexpr Scalar lambda_min_inner = static_cast<Scalar>(0.05); // About 171 degree divergence 
    Scalar lambda_safe = std::max(std::abs(lambda), lambda_min_inner);
    if (lambda < 0) {
        lambda_safe = -lambda_safe;
    }
    
    Param::Matrix43 E_ref = compute_E(q_desired);
    Param::Matrix43 E_m = compute_E(q);

    Vector3 q_aux = E_ref.transpose()*q;
    Param::Matrix3 Q = E_ref.transpose()*E_m;
    Param::Matrix3 Q_inv = Q.transpose() + Math::outerProduct(q_aux, q_aux)/lambda_safe;

    Param::Matrix3 C_BB = Q_inv*Q.transpose();
    Vector3 e_omega = omega - C_BB*omega_desired;

    // If we're in the outer loop (model -> reference), we need to 
    // treat a, the most important gain quantity in the toolbox, 
    // as the MODEL rather than the PLANT.
    Scalar a;
    if (is_outer_loop) {
        a = (a0_model - (e_omega.dot(e_omega))/4)/lambda_safe;
    } else {
        a = (a0_plant - (e_omega.dot(e_omega))/4)/lambda_safe;
    }
    return ToolBoxOutput{E_m, C_BB, a};
}

ControllerNDI::Vector4 ControllerNDI::allocateActuators(const Param::Vector3& tau_req,
                                           const Param::Vector3& tau_mtq_expected,
                                           const Param::Vector4& omega_w,
                                           Scalar mtq_comp_scale,
                                           Scalar attitude_scale,
                                           Scalar null_scale,
                                           bool apply_deadband)
{
    // Allocate torques to reaction wheels, considering desaturation torque from magnetorquers.
    Vector3 tau_to_wheels = (mtq_comp_scale * tau_mtq_expected) - (attitude_scale * tau_req);
    Vector4 tau_nom = S_pseudo * tau_to_wheels;

    // Deadband enforcement via null-space (no net body torque).
    // Build the sign pattern directly from N so it stays consistent with the
    // active spin matrix. This yields the expected ±omega_deadband hold
    // (for current geometry: [+,+,-,-] up to global sign).
    Vector4 tau_deadband = Vector4::Zero();
    if (apply_deadband) {
        Vector4 null_vec = Vector4::Zero();
        Scalar max_col_norm_sq = static_cast<Scalar>(0);
        int best_col = -1;
        for (int col = 0; col < 4; ++col) {
            Scalar norm_sq = N.col(col).squaredNorm();
            if (norm_sq > max_col_norm_sq) {
                max_col_norm_sq = norm_sq;
                best_col = col;
            }
        }

        if (best_col >= 0 && max_col_norm_sq > static_cast<Scalar>(1e-12)) {
            null_vec = N.col(best_col);
            null_vec /= null_vec.norm();

            // Fix orientation for deterministic behavior: wheel 0 prefers +deadband.
            if (null_vec(0) < static_cast<Scalar>(0)) {
                null_vec = -null_vec;
            }
        } else {
            // Fallback for degenerate projector (should not happen in normal operation).
            null_vec(0) = static_cast<Scalar>( 1);
            null_vec(1) = static_cast<Scalar>( 1);
            null_vec(2) = static_cast<Scalar>(-1);
            null_vec(3) = static_cast<Scalar>(-1);
            null_vec /= null_vec.norm();
        }

        Vector4 deadband_sign;
        for (int i = 0; i < 4; ++i) {
            deadband_sign(i) = (null_vec(i) >= static_cast<Scalar>(0))
                ? static_cast<Scalar>(1)
                : static_cast<Scalar>(-1);
        }

        Vector4 omega_deadband_err;
        for (int i = 0; i < 4; ++i) {
            omega_deadband_err(i) = deadband_sign(i) * omega_deadband - omega_w(i);
        }

        tau_deadband = k_deadband * (N * omega_deadband_err);
        // Keep bias hold subordinate to primary attitude loop.
        tau_deadband = saturateSymmetric(tau_deadband, static_cast<Scalar>(0.15) * tau_w_max);
    }

    // Null-space speed equalization: drives all wheels toward their mean.
    // Disabled when deadband is active because the deadband already governs the
    // null space — running both simultaneously causes them to partially cancel
    // (mean of [+db, +db, −db, −db] = 0, so equalization opposes the deadband).
    Vector4 tau_null = Vector4::Zero();
    if (!apply_deadband) {
        Vector4 omega_avg = Vector4::Constant(omega_w.mean());
        Vector4 omega_err = omega_avg - omega_w;
        tau_null = (null_scale * k_null) * (N * omega_err);
    }

    return tau_nom + tau_null + tau_deadband;
}

ControllerNDI::DesatOutput ControllerNDI::wheel_desaturate(const Param::Vector3& B_meas, 
                                                    const Param::Vector3& h_w) 
{
    // Desaturate reaction wheels using magnetorquers. 
    Scalar B_norm = B_meas.norm();
    if (B_norm < static_cast<Scalar>(1e-7)) {
        Vector3 m_cmd = Vector3::Zero();
        Vector3 tau_mtq_expected = Vector3::Zero();
        return DesatOutput{m_cmd, tau_mtq_expected};
    }

    // Cross product Control Law 
    Vector3 m_cmd = (k_desat / B_norm) * h_w.cross(B_meas);
    m_cmd = saturateSymmetric(m_cmd, m_max);
    Vector3 tau_mtq_expected = m_cmd.cross(B_meas);
    return DesatOutput{m_cmd, tau_mtq_expected};
}

ControllerNDI::Vector4 ControllerNDI::applyWheelSaturation(const Param::Vector4& tau_w_cmd,
                                                const Param::Vector4& omega_w)
{
    // 1. Hard torque limit
    Vector4 tau_w_sat = saturateSymmetric(tau_w_cmd, tau_w_max);

    for (int i = 0; i < 4; ++i) {
        // 2. Back-EMF reversal protection: when the commanded torque opposes the
        //    current wheel spin direction, scale the allowed limit linearly with speed.
        //    At zero speed: full tau_w_max is allowed.
        //    At omega_w_max: only tau_rev_min_frac * tau_w_max is allowed.
        //    This prevents large current spikes from back-EMF during high-speed reversals.
        bool reversing = (omega_w(i) * tau_w_sat(i) < static_cast<Scalar>(0));
        if (reversing) {
            Scalar speed_ratio = std::abs(omega_w(i)) / omega_w_max;
            Scalar rev_limit = tau_w_max * (tau_rev_min_frac + (static_cast<Scalar>(1) - tau_rev_min_frac) * (static_cast<Scalar>(1) - speed_ratio));
            tau_w_sat(i) = saturate(tau_w_sat(i), -rev_limit, rev_limit);
        }

        // 3. Speed limit: zero any torque that would spin the wheel past its physical limit
        if ((omega_w(i) >= omega_w_max) && (tau_w_sat(i) > 0)) {
            tau_w_sat(i) = 0;
        } else if ((omega_w(i) <= -omega_w_max) && (tau_w_sat(i) < 0)) {
            tau_w_sat(i) = 0;
        }
    }
    return tau_w_sat;
}

ControllerNDI::RegOutput ControllerNDI::regularize_reference(const Param::Vector4& q_m, 
                                         const ControllerNDI::Reference& reference) 
{
    // Ensure unit quaternions 
    Quat q_ref = reference.segment<4>(0);
    q_ref = q_ref / q_ref.norm();
    Vector3 omega_ref = reference.segment<3>(4);
    Vector3 alpha_ref = reference.segment<3>(7);

    // Ensure shortest rotation
    Scalar lambda = q_m.dot(q_ref);

    // Clamp numerical noise 
    lambda = saturate(lambda, static_cast<Real>(-1.0), static_cast<Real>(1.0));
    // Current error angle 
    Scalar theta_err = 2 * acos(lambda);

    // Max error angle allowed in model (derived from lambda_min_model) 
    lambda_min_model = saturate(lambda_min_model, static_cast<Real>(0.0), static_cast<Real>(1.0));
    Scalar theta_max = 2 * acos(lambda_min_model);

    // If error is already within allowed range, do nothing 
    if (abs(theta_err) <= theta_max) {
        return RegOutput{q_ref, omega_ref, alpha_ref};
    }

    // Otherwise, move that effective reference part-way along the geodesic 
    // from q_m to q_ref, so that the new error is exactly theta_max.

    // Fraction of the total geodesic length we are allowed to "see": sigma in (0,1)
    Scalar sigma = theta_max / abs(theta_err);
    Quat q_ref_eff;
    // Slerp from q_m to q_ref with parameter sigma 
    if (theta_err < static_cast<Scalar>(1e-6)) {
        // Practically identical - avoid division by zero 
        q_ref_eff = q_ref;
    } else {
        // Slerp formula for unit quaternions 
        Scalar half_theta = theta_err / 2;
        Scalar sin_half_th = sin(half_theta);
        Scalar half_theta_new = half_theta * sigma; 

        Scalar w0 = std::sin(half_theta - half_theta_new) / (sin_half_th + static_cast<Scalar>(1e-12));
        Scalar w1 = std::sin(half_theta_new) / (sin_half_th + static_cast<Scalar>(1e-12));
        
        q_ref_eff = w0 * q_m + w1 * q_ref;
        q_ref_eff = q_ref_eff / q_ref_eff.norm();
    }

    Vector3 omega_ref_eff = sigma * omega_ref;
    Vector3 alpha_ref_eff = sigma * alpha_ref;
    return RegOutput{q_ref_eff, omega_ref_eff, alpha_ref_eff};
}

bool ControllerNDI::checkWheelSaturation(const Vector4& tau_cmd, const Vector4& omega_w) {
    // Check torque magnitude saturation (95% threshold)
    bool torque_saturated = false;
    for (int i = 0; i < 4; ++i) {
        if (std::abs(tau_cmd(i)) >= tau_w_max * static_cast<Scalar>(0.95)) {
            torque_saturated = true;
            break;
        }
    }

    // Check speed saturation (95% threshold)
    bool speed_saturated = false;
    for (int i = 0; i < 4; ++i) {
        if (std::abs(omega_w(i)) >= omega_w_max * static_cast<Scalar>(0.95)) {
            speed_saturated = true;
            break;
        }
    }

    // Check back-EMF reversal clipping: if the commanded torque opposes wheel spin AND
    // the reversal protection limit is less than the commanded magnitude, the hardware
    // will clip — flag as saturated so the reference model integration freezes.
    bool reversal_saturated = false;
    for (int i = 0; i < 4; ++i) {
        bool reversing = (omega_w(i) * tau_cmd(i) < static_cast<Scalar>(0));
        if (reversing) {
            Scalar speed_ratio = std::abs(omega_w(i)) / omega_w_max;
            Scalar rev_limit = tau_w_max * (tau_rev_min_frac
                + (static_cast<Scalar>(1) - tau_rev_min_frac) * (static_cast<Scalar>(1) - speed_ratio));
            if (std::abs(tau_cmd(i)) > rev_limit) {
                reversal_saturated = true;
                break;
            }
        }
    }

    return torque_saturated || speed_saturated || reversal_saturated;
}