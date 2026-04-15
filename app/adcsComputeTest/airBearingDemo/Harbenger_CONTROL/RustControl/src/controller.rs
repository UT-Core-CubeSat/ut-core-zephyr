use nalgebra as na;
use crate::utils::quaternion;
use crate::parameters::Parameters;

pub struct Controller {
    r: na::Matrix3<f64>,           // Control input weight matrix
    q: na::Matrix6<f64>,           // State error weight matrix
    inertia: na::Matrix3<f64>,     // Inertia matrix
    gains: na::DMatrix<f64>,       // Gain scheduling table
}

impl Controller {
    pub fn new(parameters: &Parameters) -> Self {
        Controller {
            r: parameters.r.clone(),
            q: parameters.q.clone(),
            inertia: parameters.inertia.clone(),
            gains: parameters.gains.clone(),
        }
    }
    
    /// Find the nearest gains from the gain scheduling table
    pub fn find_nearest_gains(&self, quaternion: &na::Vector4<f64>) -> (na::Matrix3x6<f64>, na::Matrix3x3<f64>, na::Vector4<f64>) {
        // Extract quaternion columns from the table
        let n_rows = self.gains.nrows();
        
        // Create a vector to store similarity measures
        let mut similarities = Vec::with_capacity(n_rows);
        
        // Calculate similarity (dot product) with each quaternion in the table
        for i in 0..n_rows {
            let q_table = na::Vector4::new(
                self.gains[(i, 0)], 
                self.gains[(i, 1)], 
                self.gains[(i, 2)], 
                self.gains[(i, 3)]
            );
            
            // Calculate error quaternion
            let q_e = quaternion::invquatmultiply(quaternion, &q_table);
            
            // The quaternion with largest real part (scalar/w component) is closest
            similarities.push(q_e[0].abs());
        }
        
        // Find the index of the maximum similarity
        let nearest_idx = similarities
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(idx, _)| idx)
            .unwrap();
        
        // Extract the equilibrium state
        let x_e = na::Vector4::new(
            self.gains[(nearest_idx, 0)],
            self.gains[(nearest_idx, 1)],
            self.gains[(nearest_idx, 2)],
            self.gains[(nearest_idx, 3)]
        );
        
        // Extract K_x (state feedback gain)
        let mut k_x = na::Matrix3x6::zeros();
        for i in 0..3 {
            for j in 0..6 {
                k_x[(i, j)] = self.gains[(nearest_idx, 5 + i*6 + j)];
            }
        }
        
        // Extract K_r (reference gain)
        let mut k_r = na::Matrix3x3::zeros();
        for i in 0..3 {
            for j in 0..3 {
                k_r[(i, j)] = self.gains[(nearest_idx, 23 + i*3 + j)];
            }
        }
        
        (k_x, k_r, x_e)
    }
    
    /// Calculate LQR gains for a given equilibrium point
    pub fn calculate_gains(&self, x_e: &na::Vector4<f64>) -> (na::Matrix3x6<f64>, na::Matrix3x3<f64>) {
        let inputs_e = na::Vector3::zeros();
        
        // Convert quaternion to full state vector
        let mut x_e_full = na::Vector7::zeros();
        x_e_full[0] = x_e[0];
        x_e_full[1] = x_e[1];
        x_e_full[2] = x_e[2];
        x_e_full[3] = x_e[3];
        
        // Get linearized state space model
        let (a, b, c, d) = quaternion::modrod_state_space_fast(&x_e_full, &inputs_e, &self.inertia);
        
        // Calculate LQR gain (this would use a Rust LQR solver)
        // This is a placeholder - you would need to implement or use a library for LQR
        let k_x = self.solve_lqr(&a, &b, &self.q, &self.r);
        
        // Non-zero set point calculation
        let n_x = a.nrows();
        let n_y = c.nrows();
        let n_u = b.ncols();
        
        // Build the augmented matrix [A, B; C, D]
        let mut qpm = na::DMatrix::<f64>::zeros(n_x + n_u, n_x + n_y);
        
        // Fill in A
        for i in 0..n_x {
            for j in 0..n_x {
                qpm[(i, j)] = a[(i, j)];
            }
        }
        
        // Fill in B
        for i in 0..n_x {
            for j in 0..n_u {
                qpm[(i, n_x + j)] = b[(i, j)];
            }
        }
        
        // Fill in C
        for i in 0..n_y {
            for j in 0..n_x {
                qpm[(n_x + i, j)] = c[(i, j)];
            }
        }
        
        // Fill in D
        for i in 0..n_y {
            for j in 0..n_u {
                qpm[(n_x + i, n_x + j)] = d[(i, j)];
            }
        }
        
        // Invert the matrix
        let p = qpm.try_inverse().unwrap();
        
        // Extract P12 and P22
        let mut p12 = na::DMatrix::<f64>::zeros(n_x, n_y);
        let mut p22 = na::DMatrix::<f64>::zeros(n_u, n_y);
        
        for i in 0..n_x {
            for j in 0..n_y {
                p12[(i, j)] = p[(i, n_x + j)];
            }
        }
        
        for i in 0..n_u {
            for j in 0..n_y {
                p22[(i, j)] = p[(n_x + i, n_x + j)];
            }
        }
        
        // Calculate reference gain
        let k_r = k_x * p12 + p22;
        
        // Convert to properly sized matrices
        let k_x_sized = na::Matrix3x6::from_iterator(k_x.iter().cloned());
        let k_r_sized = na::Matrix3x3::from_iterator(k_r.iter().cloned());
        
        (k_x_sized, k_r_sized)
    }
    
    /// Update controller to calculate control inputs
    pub fn update(&self, x: &na::Vector7<f64>, r: &na::Vector7<f64>) -> na::Vector3<f64> {
        // Extract quaternions
        let q_state = na::Vector4::new(x[0], x[1], x[2], x[3]);
        let q_ref = na::Vector4::new(r[0], r[1], r[2], r[3]);
        
        // Calculate error quaternion
        let q_e = quaternion::invquatmultiply(&q_state, &q_ref);
        
        // Negate reference if taking the long way
        let q_ref_adjusted = if q_e[0] < 0.0 {
            -q_ref
        } else {
            q_ref
        };
        
        // Convert quaternions to MRPs
        let mrp = quaternion::quat2mrp(&q_state);
        let mrp_r = quaternion::quat2mrp(&q_ref_adjusted);
        
        // Extract angular velocity
        let omega = na::Vector3::new(x[4], x[5], x[6]);
        
        // Find nearest gains from the gain table
        let (k_x, k_r, x_e) = self.find_nearest_gains(&q_state);
        
        // Calculate the error MRP
        let mrp_e = quaternion::quat2mrp(&x_e);
        
        // Create state error vector [MRP error; omega]
        let mut state_error = na::Vector6::zeros();
        state_error[0] = mrp[0] - mrp_e[0];
        state_error[1] = mrp[1] - mrp_e[1];
        state_error[2] = mrp[2] - mrp_e[2];
        state_error[3] = omega[0];
        state_error[4] = omega[1];
        state_error[5] = omega[2];
        
        // Create reference error vector
        let mut ref_error = na::Vector3::zeros();
        ref_error[0] = mrp_r[0] - mrp_e[0];
        ref_error[1] = mrp_r[1] - mrp_e[1];
        ref_error[2] = mrp_r[2] - mrp_e[2];
        
        // Calculate control law: u = -K_x * state_error + K_r * ref_error
        let u = -k_x * state_error + k_r * ref_error;
        
        // Convert to RPM command
        let rpm = quaternion::tau2rpm(&u);
        
        // Apply saturation
        let mut rpm_saturated = rpm.clone();
        for i in 0..4 {
            if rpm_saturated[i] > 2000.0 {
                rpm_saturated[i] = 2000.0;
            }
            if rpm_saturated[i] < -2000.0 {
                rpm_saturated[i] = -2000.0;
            }
        }
        
        // Convert back to torque
        quaternion::rpm2tau(&rpm_saturated)
    }
    
    /// LQR solver (placeholder - would need actual implementation)
    fn solve_lqr(&self, a: &na::Matrix6<f64>, b: &na::Matrix6x3<f64>, q: &na::Matrix6<f64>, r: &na::Matrix3<f64>) -> na::DMatrix<f64> {
        // This is a placeholder for a proper LQR solver
        // In a real implementation, you would solve the Riccati equation
        
        // For now, return a matrix of reasonable values
        let mut k = na::DMatrix::<f64>::zeros(3, 6);
        
        // Fill with placeholder values
        for i in 0..3 {
            for j in 0..6 {
                k[(i, j)] = 0.1 * (i as f64 + 1.0) * (j as f64 + 1.0);
            }
        }
        
        k
    }
}