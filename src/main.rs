//! CDC-ACM serial port example using polling in a busy loop.
#![no_std]
#![no_main]

extern crate panic_halt;
use rtfm::app;
use stm32l0xx_hal::usb::{USB, UsbBus, UsbBusType};
use stm32l0xx_hal::{ prelude::*, rcc, syscfg::SYSCFG, timer, gpio::*};
use usb_device::bus;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[app(device=stm32l0xx_hal::pac, peripherals=true)]

const APP: () = {

    struct Resources {
        usb_dev: UsbDevice<'static, UsbBusType>,
	serial: SerialPort<'static, UsbBusType >,
	timer: timer::Timer<stm32l0xx_hal::pac::TIM2>,
	led: gpioa::PA5<Output<PushPull>>,
    }
    
    #[init( spawn=[handle_serial])]
    fn init(cx: init::Context) -> init::LateResources{
	static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;
        let mut rcc = cx.device.RCC.freeze(rcc::Config::hsi16());
        let mut syscfg = SYSCFG::new(cx.device.SYSCFG, &mut rcc);
        let hsi48 = rcc.enable_hsi48(&mut syscfg, cx.device.CRS);

        let gpioa = cx.device.GPIOA.split(&mut rcc);

	let led = gpioa.pa5.into_push_pull_output();

        let usb = USB::new(cx.device.USB, gpioa.pa11, gpioa.pa12, hsi48);

	*USB_BUS = Some(UsbBus::new(usb));

        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x1209, 0x0010))
            .manufacturer("Andy Goetz")
            .product("LED_CLOCKv1")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

	
	let mut timer = cx.device.TIM2.timer(100.hz(), &mut rcc);
	timer.listen();
	
	
        init::LateResources {
	    usb_dev,
            serial,
	    timer,
	    led,
        }
    }

    #[task(binds = TIM2, resources = [timer,serial,usb_dev], spawn = [toggle_led, handle_serial])]
    fn tim2_isr(cx : tim2_isr::Context) {
	static mut COUNT: u32 = 0;
	
	*COUNT+=1;

	if *COUNT > 50 {
	    *COUNT = 0;
	    cx.spawn.toggle_led().unwrap()

	}
	
	cx.resources.timer.clear_irq();		// clear interrupt flag

	cx.spawn.handle_serial().unwrap();


	
    }

    
#[task (resources=[serial, usb_dev])]
fn handle_serial(c : handle_serial::Context) {
    if !c.resources.usb_dev.poll(&mut  [ c.resources.serial]) {
	return
    }

        let mut buf = [0u8; 64];

        match c.resources.serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match c.resources.serial.write(&buf[write_offset..count]) {
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

    #[task(resources=[led])] 
    fn toggle_led(cx : toggle_led::Context) {
	cx.resources.led.toggle().unwrap();	
    }

  // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART1();
    }
    
};

