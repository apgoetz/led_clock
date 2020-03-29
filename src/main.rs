#![no_std]
#![no_main]

// pick a panicking behavior
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
// extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug};

#[entry]
fn main() -> ! {
    let DATA_addr = 0x4000c000;
    let FR_addr = 0x4000c018;
    let FR = FR_addr as *const u8;
    let DATA =  DATA_addr as *mut u8;

    unsafe {
        
    // exit QEMU
    // NOTE do not run this on hardware; it can corrupt OpenOCD state
        
        loop {
            if (*FR & 0x10)  == 0 {
                while (*FR & 0x20) != 0 {}
                let char = *DATA;
                if char == 0x03 {
                    debug::exit(debug::EXIT_SUCCESS);
                } else {
                    *DATA = char;
                }
            }
        } 
        
        loop {} // needed since main does not return
    }
}
