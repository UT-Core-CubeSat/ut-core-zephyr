#ifndef CORE_CONTROLLERNDI_HPP
#define CORE_CONTROLLERNDI_HPP

#include "core_Parameters.hpp"
#include "core_HelperFunctions.hpp"
#include "core_Saturate.hpp"

class ControllerNDI {
public:

    // Output Struct 
    struct NDIOutput {
        Param::Vector4 tau_wheel;
        Param::Vector3 tau_mtq;
        Param::Vector7 states_m;
    };
    
    // Type Aliases for readability 
    using StateVector = Param::Vector7;
    using Reference = Param::Vector10;
    using Scalar = Param::Real;
    using Vector3 = Param::Vector3;
    using Vector4 = Param::Vector4;
    using Quat = Param::Vector4;

    // Constructor
    ControllerNDI();

    NDIOutput update(const Param::Vector17& states,
                    const Param::Vector10& reference,
                    const Param::Vector29& measurements);

private:
    // Initialization Helpers
    HelperFunctions helpers;
    // Private methods 
    StateVector reference_model_dif_eq(const StateVector& states_m, 
                                       const Reference& reference);

    StateVector update_reference_model(const Reference& reference);

    Param::Matrix43 compute_E(const Quat& q);

    void calculate_gains(const Scalar& t_s_model, 
                         const Scalar& zeta_model, 
                         const Scalar& t_s_plant, 
                         const Scalar& zeta_plant);
    struct ToolBoxOutput {
        Param::Matrix43 E_m; 
        Param::Matrix3 C_BB; 
        Param::Real a;
    };
    ToolBoxOutput compute_BP_Toolbox(const StateVector& states, 
                                     const StateVector& states_desired,
                                     bool is_outer_loop); 
    Param::Vector4 allocateActuators(const Param::Vector3& tau_req,
                                      const Param::Vector3& tau_mtq_expected, 
                                      const Param::Vector4& omega_w);
    struct DesatOutput {
        Param::Vector3 m_cmd;
        Param::Vector3 tau_mtq_expected;
    };
    DesatOutput wheel_desaturate(const Param::Vector3& B_meas, 
                                 const Param::Vector3& h_w);
    Param::Vector4 applyWheelSaturation(const Param::Vector4& tau_w_cmd,
                                        const Param::Vector4& omega_w);
    struct RegOutput {
        Param::Vector4 q_ref_eff;
        Param::Vector3 omega_ref_eff;
        Param::Vector3 alpha_ref_eff;
    };
    RegOutput regularize_reference(const Param::Vector4& q_m, 
                                  const Reference& reference);
    
    bool checkWheelSaturation(const Vector4& tau_cmd, 
                              const Vector4& omega_w);

    // Private members
    Param::Matrix3 I;
    Param::Matrix34 S;
    Param::Matrix43 S_pseudo;
    Param::Matrix4 N;
    Scalar I_wheel;

    // Limits
    Scalar omega_w_max, omega_w_min;
    Scalar alpha_w_max, alpha_w_min;
    Scalar tau_w_max, tau_w_min;
    Scalar lambda_min_model;

    // Reference Model 
    StateVector x_m;

    // Time Step 
    Scalar Ts, t_end;

    // Gains
    Scalar a0_model, a1_model, a0_plant, a1_plant;
    Scalar k_desat, k_null;
    Scalar m_max, m_min;

    // Saturation anti windup 
    bool is_saturated;
};

#endif // CORE_CONTROLLERNDI_HPP