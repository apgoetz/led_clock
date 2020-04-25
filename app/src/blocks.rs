// helper blocks for realtime design.
// modeled on simulink style dataflow programming
use core::ops::{Add,Mul,Sub};
use core::marker::Copy;

// trait that represents a real time task
pub trait RTTask {
    // the cycle time this task should run at, in milliseconds
    const CYCLETIME : u16;
}


pub struct BiQuad<T> {
    w : [ T; 3],
    b : [T; 3],
    a : [T; 3]
}

impl<T> BiQuad<T> where
T : Add<Output=T> + Sub<Output=T> + Mul<Output=T> + Copy {
    
    pub fn new (init_y : T, b: [T; 3], a: [T;3]) -> BiQuad<T> {
        BiQuad {w: [init_y; 3], b:b, a : a}
    }
    pub fn step(&mut self, u : T) -> T {
        let ref mut w = self.w;
        let a = self.a;
        let b = self.b;
        let w_0 = u - a[1]*w[1] - a[2]*w[2];
        let y_0 = b[0]*w_0 + b[1]*w[1]+b[2]*w[2];
        w[2] = w[1];
        w[1] = w_0;
        y_0
    }
}

pub struct Timer32{
    counter : u32,
    alarm : u32,
}

impl Timer32 {
    // set an alarm time in milliseconds
    pub fn new<T : RTTask>(alarm  : u32) -> Timer32 {
        Timer32{alarm : alarm / T::CYCLETIME as u32, counter : 0}
    }

    pub fn reset(&mut self) {
        self.counter = 0;
    }
    
    pub fn step(&mut self) -> bool {
        if self.counter > self.alarm {
            true
        } else {
            self.counter += 1;
            false
        }
    }
}

