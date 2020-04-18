use core::ops::{Add,Mul,Sub};
use core::marker::Copy;

enum LedState {On,Off}
pub struct LEDStateMachine {
    ledfilter : BiQuad<f32>,
    timer : Timer32,
    led_state : LedState
}

impl LEDStateMachine {
    pub fn new () -> LEDStateMachine {
        let b = [ 0.00024136,  0.00048272,  0.00024136];
        let a = [ 1.        , -1.95557824,  0.95654368];
        LEDStateMachine {ledfilter : BiQuad::new(0.0,
                                                 b ,
                                                 a),
                         timer : Timer32::new(100),
                         led_state : LedState::On}
    }
    pub fn step(&mut self, button_is_pressed : bool) -> f32 {
        let alarm = self.timer.step();
        match self.led_state {
            LedState::Off => {
                if button_is_pressed == true {
                    self.led_state = LedState::On;
                    self.timer.reset();
                } else if alarm {
                    self.led_state = LedState::On;
                    self.timer.reset();
                }
                self.ledfilter.step(0.0)
            },
            LedState::On => {
                if button_is_pressed == true {
                    self.timer.reset();
                } else if alarm {
                    self.led_state = LedState::Off;
                    self.timer.reset()
                }
                self.ledfilter.step(1.0)
            }
        }
    }
}

struct BiQuad<T> {
    w : [ T; 3],
    b : [T; 3],
    a : [T; 3]
}

impl<T> BiQuad<T> where
T : Add<Output=T> + Sub<Output=T> + Mul<Output=T> + Copy {
    
    fn new (init_y : T, b: [T; 3], a: [T;3]) -> BiQuad<T> {
        BiQuad {w: [init_y; 3], b:b, a : a}
    }
    fn step(&mut self, u : T) -> T {
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

struct Timer32 {
    counter : u32,
    alarm : u32,
}

impl Timer32 {
    fn new (alarm  : u32) -> Timer32 {
        Timer32{alarm, counter : 0}
    }

    fn reset(&mut self) {
        self.counter = 0;
    }
    
    fn step(&mut self) -> bool {
        if self.counter > self.alarm {
            true
        } else {
            self.counter += 1;
            false
        }
    }
}
