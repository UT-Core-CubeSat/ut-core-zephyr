use nalgebra as na;
use std::fmt::Debug;

pub struct Integrator;

impl Integrator {
    pub fn new() -> Self {
        Integrator
    }
    
    /// RK4 integration method
    pub fn rk4<F, V>(f: F, x: &V, time: f64) -> V 
    where
        F: Fn(&V) -> V,
        V: na::Dim + Clone + Debug + na::OVector<f64, na::Dyn>,
    {
        let f1 = f(x);
        
        let mut x_half = x.clone();
        na::OVector::axpy_mut(time/2.0, &f1, &mut x_half);
        let f2 = f(&x_half);
        
        let mut x_half2 = x.clone();
        na::OVector::axpy_mut(time/2.0, &f2, &mut x_half2);
        let f3 = f(&x_half2);
        
        let mut x_full = x.clone();
        na::OVector::axpy_mut(time, &f3, &mut x_full);
        let f4 = f(&x_full);
        
        let mut result = x.clone();
        na::OVector::axpy_mut(time/6.0, &f1, &mut result);
        na::OVector::axpy_mut(time/3.0, &f2, &mut result);
        na::OVector::axpy_mut(time/3.0, &f3, &mut result);
        na::OVector::axpy_mut(time/6.0, &f4, &mut result);
        
        result
    }
    
    /// Generic integration interface
    pub fn integrate<F, V>(f: F, x: &V, time: f64) -> V 
    where
        F: Fn(&V) -> V,
        V: na::Dim + Clone + Debug + na::OVector<f64, na::Dyn>,
    {
        Self::rk4(f, x, time)
    }
}