// function to implement goertzel algorithm.
// currently optimized for ADC samples
// goertzel needs to know how big the ADC samples really are so it can normalize them before processing

use libm::{cosf};
use core::f32::consts::PI;

pub fn goertzel(x: &[f32],k:u32) -> f32
{
    let n = x.len() as f32;
    let w = 2.0*PI/n*(k as f32);
    let coeff = 2.0*cosf(w);
    let (mut q1,mut q2) = (0f32,0f32);

    for &x in x {
	let q0 = coeff*q1-q2+x;
	q2 = q1;
	q1 = q0;
    }
    q1*q1 + q2*q2 - q1*q2*coeff
}
