#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::{entry};

#[cfg(feature="qemu")]
use cortex_m_semihosting::{debug};
#[cfg(not(feature="qemu"))]
use stm32l0xx_hal; // needed if we are compiling on the device

// helper function to abstract to quit runner. Only does things if qemu is enabled . Otherwise loops
fn exit() -> ! {
    #[cfg(feature="qemu")]
    debug::exit(debug::EXIT_SUCCESS);
    loop {}
}


#[entry]
fn main() -> ! {

    let fr_reg = 0x4000c018 as *const u8;
    let data_reg =  0x4000c000 as *mut u8;

    unsafe {
        
    // exit QEMU
    // NOTE do not run this on hardware; it can corrupt OpenOCD state
        
        loop {
            if (*fr_reg & 0x10)  == 0 {
                while (*fr_reg & 0x20) != 0 {}
                let char = *data_reg;
                if char == 0x03 {
                    exit();
                } else {
                    *data_reg = char;
                }
            }
        } 
    }
}
