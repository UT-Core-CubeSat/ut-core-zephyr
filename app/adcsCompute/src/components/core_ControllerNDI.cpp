#include "core_ControllerNDI.hpp"

// Constructor 
ControllerNDI::ControllerNDI()
    : // Initialize Members
    I(Param::Spacecraft::I),
    I_wheel(Param::Actuators::I_wheel),
    S(Param::Actuators::S),
    S_pseudo(Param::Actuators::S_pseudo),
    N(Param::Actuators::N),
    omega_w_max(Param::Actuators::omega_w_max),
    omega_w_min(Param::Actuators::omega_w_min),
    tau_w_max(Param::Actuators::tau_w_max),
    tau_w_min(Param::Actuators::tau_w_min),
    lambda_min_model(Param::Controller::lambda_min_model),
    k_null(Param::Actuators::k_null),
    k_desat(Param::Actuators::k_desat),
    m_max(Param::Actuators::m_max),
    m_min(Param::Actuators::m_min),
    Ts(Param::SimTime::Ts),
    x_m(StateVector::Zero()),
    is_saturated(false)
{
    // Calculate gains 
    calculate_gains(Param::Controller::t_s_model,
                    Param::Controller::zeta_model,
                    Param::Controller::t_s_plant,
                    Param::Controller::zeta_plant);
    //x_m << Param::Orbit::InitialState.q0.segment<3>(1), 
    //       Param::Orbit::InitialState.q0(0),
    //       Param::Orbit::InitialState.omega;
    x_m(0) = 0; x_m(1) = 0; x_m(2) = 0; x_m(3) = 1; // identity quat
    x_m(4) = 0; x_m(5) = 0; x_m(6) = 0; // zero body rates

}

ControllerNDI::NDIOutput ControllerNDI::update(const Param::Vector17& states,
                                     const Param::Vector10& reference,
                                     const Param::Vector29& measurements) 
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
    q(0) = states(7);
    q(1) = states(8);
    q(2) = states(9);
    q(3) = states(6);
    q.normalize();
    Vector3 omega = states.segment<3>(10);
    Vector4 omega_w = states.segment<4>(13);

    // Outer Loop (Model -> Reference)
    StateVector x_model = update_reference_model(NDI_reference);
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

    // Compute torque from Euler's equation
    Vector3 tau_NDI = I*omega_dot + omega.cross(I*omega);

    // Get desat torque
    Vector3 h_w = S * (I_wheel * omega_w);
    Vector3 B_meas = measurements.segment<3>(16);
    DesatOutput desat_out = wheel_desaturate(B_meas, h_w);
    Vector3 tau_mtq = desat_out.tau_mtq_expected;

    // Convert to wheel torques, plus desat 
    Vector4 tau_tilde = allocateActuators(tau_NDI, tau_mtq, omega_w);
    // Anti windup: Check for saturation for NEXT timestep
    is_saturated = checkWheelSaturation(tau_tilde, omega_w);
    // Apply Saturation
    Vector4 tau_wheel = applyWheelSaturation(tau_tilde, omega_w);
    
    // Pack up model 
    Quat q_m_out;
    q_m_out(0) = x_model(3);
    q_m_out(1) = x_model(0);
    q_m_out(2) = x_model(1);
    q_m_out(3) = x_model(2);
    StateVector states_m;
    states_m.setSegment(0, q_m_out);
    states_m.setSegment(4, omega_m);
    return NDIOutput{tau_wheel, tau_mtq, states_m};
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

ControllerNDI::StateVector ControllerNDI::update_reference_model(const Reference& reference) 
{
    // Integrate ODE using Runge-Kutta RK4 method 
    StateVector x_m_dot_1 = reference_model_dif_eq(x_m, reference);
    StateVector x_m_dot_2 = reference_model_dif_eq(x_m + (Ts/2)*x_m_dot_1, reference);
    StateVector x_m_dot_3 = reference_model_dif_eq(x_m + (Ts/2)*x_m_dot_2, reference);
    StateVector x_m_dot_4 = reference_model_dif_eq(x_m + Ts*x_m_dot_3, reference);
    if (is_saturated) {
        x_m.setSegment(0, x_m.segment<4>(0) + (Ts/6)*(x_m_dot_1.segment<4>(0) + 2*x_m_dot_2.segment<4>(0) + 2*x_m_dot_3.segment<4>(0) + x_m_dot_4.segment<4>(0)));
        x_m.setSegment(0, x_m.segment<4>(0).normalized());
        return x_m;
    }
    x_m += (Ts/6)*(x_m_dot_1 + 2*x_m_dot_2 + 2*x_m_dot_3 + x_m_dot_4);
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
    q = q / q.norm();
    Vector3 omega = states.segment<3>(4);
    Quat q_desired = states_desired.segment<4>(0);
    q_desired = q_desired / q_desired.norm();
    Vector3 omega_desired = states_desired.segment<3>(4);

    Scalar lambda = q.dot(q_desired);

    // Clamp lambda to prevent division by near-zero 
    // Safety net for edge cases where the model diverges enough from the plant 
    constexpr Scalar lambda_min_inner = 0.05; // About 171 degree divergence 
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
                                           const Param::Vector4& omega_w) 
{
    // Allocate torques to reaction wheels, considering desaturation torque from magnetorquers. 
    Vector3 tau_to_wheels = tau_mtq_expected - tau_req;
    Vector4 tau_nom = S_pseudo * tau_to_wheels;

    Vector4 omega_avg = Vector4::Constant(omega_w.mean());
    Vector4 omega_err = omega_avg - omega_w;

    Vector4 tau_null = k_null * (N * omega_err);

    return tau_nom + tau_null;
}

ControllerNDI::DesatOutput ControllerNDI::wheel_desaturate(const Param::Vector3& B_meas, 
                                                    const Param::Vector3& h_w) 
{
    // Desaturate reaction wheels using magnetorquers. 
    Scalar B_norm = B_meas.norm();
    if (B_norm < 1e-7) {
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
    // Apply saturation limits to reaction wheel torques. 
    Vector4 tau_w_sat = saturateSymmetric(tau_w_cmd, tau_w_max);

    for (int i = 0; i < 4; ++i) {
        // Check for wheel speed limits 
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
    lambda = std::clamp(lambda, -1.0, 1.0);
    // Current error angle 
    Scalar theta_err = 2 * acos(lambda);

    // Max error angle allowed in model (derived from lambda_min_model) 
    lambda_min_model = std::clamp(lambda_min_model, 0.0, 1.0);
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
    if (theta_err < 1e-6) {
        // Practically identical - avoid division by zero 
        q_ref_eff = q_ref;
    } else {
        // Slerp formula for unit quaternions 
        Scalar half_theta = theta_err / 2;
        Scalar sin_half_th = sin(half_theta);
        Scalar half_theta_new = half_theta * sigma; 

        Scalar w0 = sin(half_theta - half_theta_new) / (sin_half_th + 1e-12);
        Scalar w1 = sin(half_theta_new) / (sin_half_th + 1e-12);
        
        q_ref_eff = w0 * q_m + w1 * q_ref;
        q_ref_eff = q_ref_eff / q_ref_eff.norm();
    }

    Vector3 omega_ref_eff = sigma * omega_ref;
    Vector3 alpha_ref_eff = sigma * alpha_ref;
    return RegOutput{q_ref_eff, omega_ref_eff, alpha_ref_eff};
}

bool ControllerNDI::checkWheelSaturation(const Vector4& tau_cmd, const Vector4& omega_w) {
    // Check torque saturation
    bool torque_saturated = false;
    for (int i = 0; i < 4; ++i) {
        if (std::abs(tau_cmd(i)) >= tau_w_max * 0.95) {  // 95% threshold
            torque_saturated = true;
            break;
        }
    }
    
    // Check speed saturation
    bool speed_saturated = false;
    for (int i = 0; i < 4; ++i) {
        if (std::abs(omega_w(i)) >= omega_w_max * 0.95) {  // 95% threshold
            speed_saturated = true;
            break;
        }
    }
    
    return torque_saturated || speed_saturated;
}