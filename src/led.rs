use crate::blocks;
use usb_device::device::UsbDeviceState;
use core::fmt::Debug;

#[derive(Debug)]
pub enum LedState {On,Off, Suspend}

pub struct LEDInput {
    pub button_is_pressed : bool,
    pub usbstate : UsbDeviceState    ,
}
pub struct LEDOutput {
    pub led_level : f32,
    pub go_to_sleep : bool,
}

pub struct LEDStateMachine {
    ledfilter : blocks::BiQuad<f32>,
    timer : blocks::Timer32,
    pub led_state : LedState
}

impl blocks::RTTask for LEDStateMachine {
    const CYCLETIME: u16 = 10;  // expect to cycle every 10ms
}

impl LEDStateMachine {

    pub fn new () -> LEDStateMachine {
        let b = [ 0.00024136,  0.00048272,  0.00024136];
        let a = [ 1.        , -1.95557824,  0.95654368];
        LEDStateMachine {ledfilter : blocks::BiQuad::new(0.0,
                                                 b ,
                                                         a),
                         // 1 second time
                         timer : blocks::Timer32::new::<Self>(1000),
                         led_state : LedState::On}
    }
    
    pub fn step(&mut self, input : &LEDInput) -> LEDOutput {
        let &LEDInput {button_is_pressed, usbstate} = input;
        let alarm = self.timer.step();
        match self.led_state {
            LedState::Off => {
                if button_is_pressed == true {
                    self.led_state = LedState::On;
                    self.timer.reset();
                } else if alarm && usbstate == UsbDeviceState::Suspend {
                    self.led_state = LedState::Suspend;
                }
                LEDOutput{led_level:self.ledfilter.step(0.0), go_to_sleep:false}
            },
            LedState::On => {
                if button_is_pressed == true {
                    self.timer.reset();
                } else if alarm {
                    self.led_state = LedState::Off;
                    self.timer.reset()
                }
                LEDOutput{led_level:self.ledfilter.step(1.0), go_to_sleep:false}
            }
            LedState::Suspend => {
                if button_is_pressed {
                    self.led_state = LedState::On;
                }
                LEDOutput{led_level:self.ledfilter.step(0.0), go_to_sleep:true}
            }
        }
    }
}

