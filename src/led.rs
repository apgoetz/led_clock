use crate::blocks;

enum LedState {On,Off}

pub struct LEDStateMachine {
    ledfilter : blocks::BiQuad<f32>,
    timer : blocks::Timer32,
    led_state : LedState
}

impl LEDStateMachine {
    pub fn new () -> LEDStateMachine {
        let b = [ 0.00024136,  0.00048272,  0.00024136];
        let a = [ 1.        , -1.95557824,  0.95654368];
        LEDStateMachine {ledfilter : blocks::BiQuad::new(0.0,
                                                 b ,
                                                 a),
                         timer : blocks::Timer32::new(100),
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

