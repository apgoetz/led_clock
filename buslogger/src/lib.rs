#![cfg_attr(not(feature = "std"), no_std)]
#[cfg(feature = "std")]
use std::fmt;
#[cfg(not(feature = "std"))]
use core::fmt;

use embedded_hal::blocking::i2c;

//mod flog;

/// A wrapper around an embedded_hal bus peripheral that traces each
/// read and write call and prints the raw bytes that were sent or
/// received. It will also pretty-print the error result if the
/// underlying request failed.
///
/// Currently, this requires that the error type of the underlying peripheral
/// implements std::fmt::Debug, which most of them seem to do. 

pub struct BusLogger<W : fmt::Write,T> (
    W, T
);

impl<W: fmt::Write, T> BusLogger<W,T> {
    pub fn new(w : W, t : T) -> BusLogger<W,T> {
        BusLogger(w,t)
    }
    
}

impl <W : fmt::Write, T : i2c::Read> i2c::Read for BusLogger<W,T> where
    <T as i2c::Read>::Error : fmt::Debug
{
    type Error = T::Error;

    fn read(&mut self, address: u8, buffer: &mut[u8]) -> Result<(), Self::Error> {
        write!(self.0, "a[{:02x}] ", address).ok();
        let result = self.1.read(address,buffer);
        match result {
            Err(e) => { writeln!(self.0, "{:?}\r", e).ok(); Err(e)}
            _ => {writeln!(self.0, "r{:02x?}\r", buffer).ok(); Ok(())}
        }
    }
}

impl <W : fmt::Write, T : i2c::Write> i2c::Write for BusLogger<W,T> where
    <T as i2c::Write>::Error : fmt::Debug
{
    type Error = T::Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        write!(self.0, "a[{:02x}] ", addr).ok();
        write!(self.0, "w{:02x?} ", bytes).ok();
        let result = self.1.write(addr,bytes);
        match result {
            Err(e) => { writeln!(self.0, "{:?}\r", e).ok(); Err(e)}
            _ => {writeln!(self.0, "\r").ok(); Ok(())}
        }
    }
}

impl <W : fmt::Write, T : i2c::WriteRead> i2c::WriteRead for BusLogger<W,T> where
    <T as i2c::WriteRead>::Error : fmt::Debug
{
    type Error = T::Error;

    fn write_read(&mut self, address: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        write!(self.0, "a[{:02x}] ", address).ok();
        write!(self.0, "w{:02x?} ", bytes).ok();
        let result = self.1.write_read(address,bytes,buffer);
        match result {
            Err(e) => { writeln!(self.0, "{:?}\r", e).ok(); Err(e)}
            _ => {writeln!(self.0, "r{:02x?}\r",buffer).ok(); Ok(())}
        }
    }
}
