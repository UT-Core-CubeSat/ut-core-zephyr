use nalgebra as na;
use crate::integrator::Integrator;
use crate::utils::quaternion;

pub struct Dynamics {
    inertia: na::Matrix3<f64>,
}

impl Dynamics {
    pub fn new(parameters: &crate::parameters::Parameters) -> Self {
        Dynamics {
            inertia: parameters.inertia.clone(),
        }
    }
    
    pub fn moments(&self, u: &na::Vector3<f64>) -> na::Vector3<f64> {
        // In this simple case, input torques are directly applied
        u.clone()
    }
    
    pub fn kinetics(&self, x: &na::Vector7<f64>, u: &na::Vector3<f64>) -> na::Vector7<f64> {
        // Unpack state
        let q = na::Vector4::new(x[0], x[1], x[2], x[3]);
        let omega = na::Vector3::new(x[4], x[5], x[6]);
        
        // Calculate quaternion derivative: q_dot = 0.5 * [0; omega] * q
        let omega_quat = na::Vector4::new(0.0, omega[0], omega[1], omega[2]);
        let q_dot = quaternion::quatmultiply(&omega_quat, &q).scale(0.5);
        
        // Calculate angular acceleration: omega_dot = I^-1 * (u - omega × (I * omega))
        let i_omega = self.inertia * omega;
        let cross_term = omega.cross(&i_omega);
        let omega_dot = self.inertia.try_inverse().unwrap() * (u - cross_term);
        
        // Combine into state derivative
        let mut x_dot = na::Vector7::zeros();
        x_dot[0] = q_dot[0];
        x_dot[1] = q_dot[1];
        x_dot[2] = q_dot[2];
        x_dot[3] = q_dot[3];
        x_dot[4] = omega_dot[0];
        x_dot[5] = omega_dot[1];
        x_dot[6] = omega_dot[2];
        
        x_dot
    }
    
    pub fn update(&self, x: &na::Vector7<f64>, u: &na::Vector3<f64>, time: f64) -> na::Vector7<f64> {
        // Apply control torques
        let tau = self.moments(u);
        
        // Create a closure for the integrator
        let dynamics_fn = |state: &na::Vector7<f64>| self.kinetics(state, &tau);
        
        // Propagate the model
        let mut x_new = Integrator::rk4(dynamics_fn, x, time);
        
        // Normalize quaternion
        let quat_norm = na::Vector4::new(x_new[0], x_new[1], x_new[2], x_new[3]).norm();
        x_new[0] /= quat_norm;
        x_new[1] /= quat_norm;
        x_new[2] /= quat_norm;
        x_new[3] /= quat_norm;
        
        x_new
    }
}