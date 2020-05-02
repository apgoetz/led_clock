//base software for led clock. Contains all code to talk to hardware

#![no_std]
#![no_main]

extern crate panic_halt;
use rtfm::app;
use stm32l0xx_hal::usb::{USB, UsbBus, UsbBusType};
use stm32l0xx_hal::{serial, prelude::*, rcc, syscfg::SYSCFG, timer, gpio::*, pwm};
use stm32l0xx_hal::serial::Serial1Ext;
use stm32l0::stm32l0x3;
use usb_device::{bus,prelude::*, device};
use usbd_serial::{SerialPort, USB_CLASS_CDC, DefaultBufferStore};
use core::fmt;
use core::fmt::Write;

use app::blocks::RTTask;
use app::led;
use app::humbutton::HumButton;


use stm32l0xx_hal::adc;


use stm32l0xx_hal::i2c;
use mfxstm32l152::{MFX, NbShunt, DelayUnit};
use stm32l0xx_hal::delay;

// typealias to save on typing the mfx device object
type MFXDriver = MFX<i2c::I2c<stm32l0x3::I2C1, gpiob::PB9<Output<OpenDrain>>, gpiob::PB8<Output<OpenDrain>>>, gpioa::PA1<Output<PushPull>>, delay::Delay>;

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

        // used to read AC hum
        adc: adc::Adc<adc::Ready>,
        // io pin for  reading AC hum
        ain: gpioa::PA4<Analog>,
        // uart for debugging
        uart : serial::Tx<stm32l0x3::USART1>,
        // usb device for handing usb port
        usb_dev: UsbDevice<'static, UsbBusType>,
        // CDC-ACM object for debugging
	serial: SerialWrapper<'static, UsbBusType >,
        // hardware timer used to schedule RT tasks (1kHz)
	timer: timer::Timer<stm32l0xx_hal::pac::TIM21>,
        // io pin for LED control (pwm pin)
	led_pin : pwm::Pwm<stm32l0xx_hal::pac::TIM2, pwm::C1, pwm::Assigned<gpioa::PA5<Analog>>>,
        // io pin for hardware ui button
        hw_button : gpioa::PA0<Input<Floating>>,

        // global state to indicate if button is pressed (used to
        // communicate between hum button driver and main code
	button_pressed : bool,

        // driver for multifunction expander
        mfx : MFXDriver,
        
    }
    
    #[init]
    fn init(cx: init::Context) -> init::LateResources{
	static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;
        let mut rcc = cx.device.RCC.freeze(rcc::Config::hsi16());
        let mut syscfg = SYSCFG::new(cx.device.SYSCFG, &mut rcc);
        let hsi48 = rcc.enable_hsi48(&mut syscfg, cx.device.CRS);

        let gpioa = cx.device.GPIOA.split(&mut rcc);
        let gpiob = cx.device.GPIOB.split(&mut rcc);

	let pwm = pwm::Timer::new(cx.device.TIM2, 10.khz(), &mut rcc);
	let mut led_pin = pwm.channel1.assign(gpioa.pa5);
        led_pin.enable();
        let hw_button = gpioa.pa0.into_floating_input();
	
        let usb = USB::new(cx.device.USB, gpioa.pa11, gpioa.pa12, hsi48);

	*USB_BUS = Some(UsbBus::new(usb));

        
        let serial = SerialWrapper(SerialPort::new(USB_BUS.as_ref().unwrap()));

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x1209, 0x0010))
            .manufacturer("Andy Goetz")
            .product("LED_CLOCKv1")
            .serial_number("asdf")
            .device_class(USB_CLASS_CDC)
            .build();

	let timer = cx.device.TIM21.timer(1000.hz(), &mut rcc);
	//timer.listen();

        

        
        let txpin = gpiob.pb6;
        let rxpin = gpiob.pb7;
        let uart = cx.device.USART1.usart(txpin, rxpin,serial::Config::default(), &mut rcc).unwrap();
        let (mut uart,_) = uart.split();


        let adc = cx.device.ADC.constrain(&mut rcc);
        let ain = gpioa.pa4.into_analog();

        let sda = gpiob.pb9.into_open_drain_output();
        let scl = gpiob.pb8.into_open_drain_output();
        let i2c = cx.device.I2C1.i2c(sda, scl, 150.khz(), &mut rcc);
        let wakeup  = gpioa.pa1.into_push_pull_output();
//        let wakeup = OldOutputPin::new(wakeup); // convert into old-style pin
        let delay = delay::Delay::new(cx.core.SYST, rcc.clocks);
        let mfx = MFX::new(i2c, wakeup, delay, 0x42).unwrap();
        
        
        writeln!(uart, "finished init\r").unwrap();
        init::LateResources {
            uart,
	    usb_dev,
            serial,
	    timer,
	    led_pin,
            hw_button,
            ain,
            adc,
	    button_pressed : false,
            mfx,
        }
    }


    #[idle(resources=[mfx,uart])]
    fn idle(cx : idle::Context ) -> ! {
    	let idle::Resources {mut mfx,mut uart} = cx.resources;

        loop {
	match mfx.wakup() {
	    Err(e) => {
		uart.lock(|s| {writeln!(s, "wakeup error {:?}\r",e).ok()});
 	    }
	    _ => break
	};
        }
    	 uart.lock(|s| {
    	     writeln!(s, "finished mfx setup\r").ok();
    	    });

	uart.lock(|s| {
	    writeln!(s, "errcode {:?}\r",mfx.error_code()).ok();
	    writeln!(s, "fw_ver {:?}\r",mfx.firmware_version()).ok();
	});


        
	setup_mfx(&mut mfx);	
    	loop {
    	    uart.lock(|s| {
    		writeln!(s, "starting idd meas:\r").ok();
    	    });
    	    mfx.idd_start().unwrap();

            mfx.wait_for_measurement().ok();
            
    	    let idd = mfx.idd_get_value().unwrap();
    	    let error = mfx.error_code().unwrap();
    	    uart.lock(|s| {
    		writeln!(s, "IDD: {} nA, Error: {}\r", idd,error).ok();
    	    });
    	}
    }
    
    
    // kicks off all of the periodic tasks. If any of them are still
    // running when the interrupt fires again, the spawn call will fail and we panic
    // this thread must have the highest priority, because it implements SW watchdog
    #[task(binds = TIM21, resources=[timer], spawn = [run_led,run_adc], priority=2)]
    fn scheduler(cx : scheduler::Context) {
        static mut DIVIDER : u16 = 0;

	cx.resources.timer.clear_irq();		// clear interrupt flag, so we dont execute continuously
        // kick off tasks at their samplerate
        if *DIVIDER % led::LEDStateMachine::CYCLETIME == 0 {
            cx.spawn.run_led().unwrap();
        }
        if *DIVIDER % 5 == 0 {
            cx.spawn.run_adc().unwrap();
        }
        
        
        *DIVIDER += 1;
    }

    // USB ISR (stm32l0xx only has one interrupt for all the things)
    // must call usb poll in here to handle usb traffic
    #[task (binds=USB, resources=[serial,usb_dev, uart])]
    fn usb(cx:usb::Context) {
        static mut OLDSTATE : Option<device::UsbDeviceState> = None;
	let usb_dev = cx.resources.usb_dev;
        let serial = cx.resources.serial;

	// do the polling of the USB state
	usb_dev.poll(&mut  [ &mut serial.0]);
	
        let state = usb_dev.state();
        if Some(state) != *OLDSTATE {
            *OLDSTATE = Some(state);
            writeln!(cx.resources.uart, "{}\r",
            match state {
                usb_device::device::UsbDeviceState::Default => "def",
                usb_device::device::UsbDeviceState::Addressed => "addr",
                usb_device::device::UsbDeviceState::Configured => "conf",
                usb_device::device::UsbDeviceState::Suspend => "susp",
            }).ok();

            // Note: stm32_usb handles FSUSP and LPMOODE if a
            // suspend/resume event occurs during the poll() call above
        }
    }

    #[task(resources=[adc,ain,serial,hw_button, button_pressed])] 
    fn run_adc(cx : run_adc::Context) {
	static mut BUTTON : HumButton = HumButton::new();


        let run_adc::Resources {adc,ain,serial,hw_button, button_pressed} = cx.resources;
        let rawval : u16 = adc.read(ain).unwrap();
	let val = (rawval as f32) / 4096.0;

	*button_pressed = BUTTON.step(&val);
	*button_pressed |= hw_button.is_high().unwrap(); 
	
        writeln!(serial, ": {}\r", rawval).ok();        
    }
    
    #[task(resources=[led_pin, button_pressed, usb_dev])] 
    fn run_led(cx : run_led::Context) {
	static mut LED_SM : Option<led::LEDStateMachine> = None;

	if let None = *LED_SM {
	    *LED_SM = Some(led::LEDStateMachine::new());
	}

	
        let run_led::Resources {led_pin,button_pressed,usb_dev} = cx.resources;
        let max = led_pin.get_max_duty() as f32;
        let usbstate = usb_dev.state();
	let button_pressed = *button_pressed;
        let output = LED_SM.as_mut().unwrap().step(&led::LEDInput{button_pressed, usbstate});
        let duty = max*output.led_level;
        if duty < 0.0 {
            led_pin.set_duty(0)
        } else {
            led_pin.set_duty(duty as u16);
        }
    }

  // Interrupt handlers used to dispatch software tasks
    extern "C" {
        fn USART2();
    }
    
};


// helper function to wrap setting up of the MFX device
fn setup_mfx( mfx : &mut MFXDriver) {
    mfx.set_idd_ctrl(false, false, NbShunt::SHUNT_NB_4).unwrap();
    mfx.set_idd_gain(4990).unwrap();    // gain is 4990
    mfx.set_idd_vdd_min(2000).unwrap(); // In milivolt
    mfx.set_idd_pre_delay(DelayUnit::TIME_20_MS, 1).unwrap(); // 20ms delay
    mfx.set_idd_shunt0(1000, 149).unwrap();
    mfx.set_idd_shunt1(24, 149).unwrap();
    mfx.set_idd_shunt2(620, 149).unwrap();
    mfx.set_idd_shunt3(0, 0).unwrap();
    mfx.set_idd_shunt4(10000, 255).unwrap();
    mfx.set_idd_nb_measurment(1).unwrap(); // number of measurements to take 
    mfx.set_idd_meas_delta_delay(DelayUnit::TIME_5_MS, 1).unwrap(); // delay 5ms between samples
}
