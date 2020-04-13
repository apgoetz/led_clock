//CDC-ACM serial port example using polling in a busy loop.
#![no_std]
#![no_main]

extern crate panic_halt;
use rtfm::app;
use stm32l0xx_hal::usb::{USB, UsbBus, UsbBusType};
use stm32l0xx_hal::{ prelude::*, rcc, syscfg::SYSCFG, timer, gpio::*, pwm};
use usb_device::bus;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC, DefaultBufferStore};
use core::fmt;
use core::fmt::Write;


// we need to wrap the serialport interface in a newtype in order to
// use formatting macros with the port.
//
// We would prefer to implement std::io::Write, but that is not
// possible right now with no_std embedded rust, so instead of doing
// that, we are going to implement core::fmt::Write
pub struct SerialWrapper<'a, B,R = DefaultBufferStore,W = DefaultBufferStore>(SerialPort<'a, B,R,W >) where
    B: bus::UsbBus,
    R: core::borrow::BorrowMut<[u8]>,
    W: core::borrow::BorrowMut<[u8]>;

impl<'a, B,R,W> fmt::Write for SerialWrapper <'a, B,R,W> where
    B: bus::UsbBus,
    R: core::borrow::BorrowMut<[u8]>,
    W: core::borrow::BorrowMut<[u8]>
{

    // technically, we are supposed to block until the ENTIRE string
    // has been written. But that doesnt make sense in this
    // application, so instead we are going to _try_ to write the
    // string, and if we cannot write the whole thing, we return an
    // error. The result type allowed in the fmt::Write doesnt return
    // number of bytes written, but that is OK for now, since we only
    // intend this write interface to be used for debugging stuff.
    fn write_str(&mut self, s: &str) -> fmt::Result {

	match self.0.write(s.as_bytes()) {
	    Ok(len) => { if len == s.len() { Ok(()) } else {Err(fmt::Error) } },
	    _ => Err(fmt::Error)
	}
    }
}

#[app(device=stm32l0xx_hal::pac, peripherals=true)]
const APP: () = {

    struct Resources {
        usb_dev: UsbDevice<'static, UsbBusType>,
	serial: SerialWrapper<'static, UsbBusType >,
	timer: timer::Timer<stm32l0xx_hal::pac::TIM21>,
	led : pwm::Pwm<stm32l0xx_hal::pac::TIM2, pwm::C1, pwm::Assigned<gpioa::PA5<Analog>>>,
        button : gpioa::PA0<Input<Floating>>,//pa0
    }
    
    #[init]
    fn init(cx: init::Context) -> init::LateResources{
	static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;
        let mut rcc = cx.device.RCC.freeze(rcc::Config::hsi16());
        let mut syscfg = SYSCFG::new(cx.device.SYSCFG, &mut rcc);
        let hsi48 = rcc.enable_hsi48(&mut syscfg, cx.device.CRS);

        let gpioa = cx.device.GPIOA.split(&mut rcc);

	let pwm = pwm::Timer::new(cx.device.TIM2, 10.khz(), &mut rcc);
	let led = pwm.channel1.assign(gpioa.pa5);

        let button = gpioa.pa0.into_floating_input();
	
        let usb = USB::new(cx.device.USB, gpioa.pa11, gpioa.pa12, hsi48);

	*USB_BUS = Some(UsbBus::new(usb));

        let serial = SerialWrapper(SerialPort::new(USB_BUS.as_ref().unwrap()));

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x1209, 0x0010))
            .manufacturer("Andy Goetz")
            .product("LED_CLOCKv1")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

	let mut timer = cx.device.TIM21.timer(100.hz(), &mut rcc);
	timer.listen();
        
        init::LateResources {
	    usb_dev,
            serial,
	    timer,
	    led,
            button,
        }
    }

    // kicks off all of the periodic tasks. If any of them are still
    // running when the interrupt fires again, the spawn call will fail and we panic
    // this thread must have the highest priority
    #[task(binds = TIM21, resources=[timer], spawn = [run_led] , priority=2)]
    fn scheduler(cx : scheduler::Context) {

	cx.spawn.run_led().unwrap();
	cx.resources.timer.clear_irq();		// clear interrupt flag, so we dont execute continuously

    }

    // USB ISR (stm32l0xx only has one interrupt for all the things)
    // must call usb poll in here to handle usb traffic
    #[task (binds=USB, resources=[serial,usb_dev], spawn=[handle_serial])]
    fn usb(cx:usb::Context) {
	let usb_dev = cx.resources.usb_dev;
        let spawn = cx.spawn;
        let serial = cx.resources.serial;
	if usb_dev.poll(&mut  [ &mut serial.0])  {
	    spawn.handle_serial().unwrap();
	}
    }
    
    #[task (resources=[serial])]
    fn handle_serial(cx : handle_serial::Context) {

    	let mut buf = [0u8; 64];
        let serial = cx.resources.serial;
    	match serial.0.read(&mut buf) {
    	    Ok(count) if count > 0 => {
    		// Echo back in upper case
    		for c in buf[0..count].iter_mut() {
    		    if 0x61 <= *c && *c <= 0x7a {
    			*c &= !0x20;
    		    }
    		}

    		let mut write_offset = 0;
    		while write_offset < count {
                   match serial.0.write(&buf[write_offset..count]) {
    			Ok(len) if len > 0 => { 
    			   write_offset += len;
    			}
    			_ => {}
    		    }
    		}
    	    }
    	    _ => {}
    	}
    }

    #[task(resources=[serial,led, button])] 
    fn run_led(cx : run_led::Context) {
        static mut DUTY : u16 = 0;
        
        let led = cx.resources.led;
        let max = led.get_max_duty();
        let delta =  max / 100;

        *DUTY += delta;
        
        if *DUTY > max {
            *DUTY = 0;
        }
        
	led.set_duty(*DUTY);
        writeln!(cx.resources.serial, "duty: {}\r", *DUTY).ok();
    }

  // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART1();
    }
    
};

