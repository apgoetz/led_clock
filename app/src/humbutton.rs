use crate::blocks;
use crate::goertzel::goertzel;
use core::f32;
const N :usize = 20;
const FS :usize = 200;
const K : u32 = 6;

pub type  HumButtonInput = f32;
pub type  HumButtonOutput = bool;


pub struct HumButton {
    buffer : [f32 ; N],
    ispressed : bool,
    maxreading : f32,
    minreading : f32,
    curval : usize,
}

fn max(a:f32,b:f32) -> f32 {
    if a > b {
	a
    } else {
	b
    }
}

fn min(a:f32,b:f32) -> f32 {
    if a > b {
	b
    } else {
	a
    }
}


impl HumButton {
    pub const fn new() -> HumButton {
	HumButton {buffer : [0.0 ; N],
		   ispressed : false,
		   maxreading : 40.0, // determined by calibration
		   minreading : f32::MAX,
		   curval : 0}
    }
    pub fn step(&mut self, input : &HumButtonInput) -> HumButtonOutput {
	self.buffer[self.curval] = *input;
	self.curval += 1;
	if self.curval < N {
	    return self.ispressed;
	}

	self.curval = 0;
	let f_x = goertzel(&self.buffer, K);
	self.maxreading = max(self.maxreading,f_x);
	self.minreading = min(self.minreading,f_x);
	let threshold = (self.maxreading + self.minreading) / 2.0;
	
        self.ispressed = if self.ispressed {
            if f_x < threshold*0.7 {
                false
	    } else {
		true
	    }
	} else {
            if f_x >= threshold * 1.2 {
                true
	    } else {
                false
	    }
	};

	self.ispressed
    }
}

impl blocks::RTTask for HumButton {
    const CYCLETIME: u16 = (1000/FS) as u16;  // expect to cycle every 10ms
}

