#![no_std]
#![no_main]

mod pdm;

use core::sync::atomic::{AtomicU32, Ordering};
use arrayvec::ArrayString;
use cortex_m;
use cortex_m::peripheral::SYST;
use embedded_hal::digital::v2::OutputPin;
use max31855::Error;
use panic_halt as _;
use stm32f4xx_hal::rcc::Clocks;
use stm32f4xx_hal::gpio::{Alternate, Output};

pub static TIME: AtomicU32 = AtomicU32::new(0);
pub const SYS_CK_MHZ: u32 = 144;
pub const SYST_RELOAD: u32 = (SYS_CK_MHZ * 1000) - 1;


#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use core::fmt::{Debug, Write};
    use core::sync::atomic::Ordering;
    use arrayvec::ArrayString;
    use embedded_hal::blocking::delay::DelayUs;
    use embedded_hal::blocking::spi::Transfer;
    use embedded_hal::digital::v2::OutputPin;
    //use max31855::{FullResult, Unit};
    use stm32f4xx_hal::gpio::{Alternate, Output};
    use stm32f4xx_hal::otg_fs::{USB, UsbBus};
    use stm32f4xx_hal::prelude::*;
    use stm32f4xx_hal::spi::{Mode, Phase, Polarity, Spi};
    use systick_monotonic::fugit::Duration;
    use systick_monotonic::Systick;
    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::prelude::*;
    use crate::pdm::Pdm;
    use crate::{get_time, SYS_CK_MHZ, systick_init};

    //mod pdm;

    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

    #[derive(Debug)]
    pub enum E {
        /// temperatuuri lugemise viga
        NoConnectionError,
    }

    #[derive(Debug)]
    pub struct FullResult {
        /// The temperature of the thermocouple
        pub temp: f32,
        /// The temperature of the MAX31855 reference junction
        pub inttemp: f32,
    }

    #[shared]
    struct Shared {
        serial: usbd_serial::SerialPort<'static, UsbBus<USB>>,
    }

    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        spi: Spi<stm32f4xx_hal::pac::SPI1, (stm32f4xx_hal::gpio::Pin<'A', 5_u8, Alternate<5_u8>>, stm32f4xx_hal::gpio::Pin<'A', 6_u8, Alternate<5_u8>>, stm32f4xx_hal::gpio::Pin<'A', 7_u8, Alternate<5_u8>>)>,
        cs1: stm32f4xx_hal::gpio::Pin<'A', 4_u8, Output>,
        cs2: stm32f4xx_hal::gpio::Pin<'A', 8_u8, Output>,
        cs3: stm32f4xx_hal::gpio::Pin<'A', 9_u8, Output>,
        kyte1: stm32f4xx_hal::gpio::Pin<'C', 0_u8, Output>,
        kyte2: stm32f4xx_hal::gpio::Pin<'C', 1_u8, Output>,
        kyte3: stm32f4xx_hal::gpio::Pin<'C', 2_u8, Output>,
        peltier1: stm32f4xx_hal::gpio::Pin<'A', 1_u8, Output>,
        peltier2: stm32f4xx_hal::gpio::Pin<'A', 2_u8, Output>,
        delay: stm32f4xx_hal::timer::Delay<stm32f4xx_hal::pac::TIM5, 1000000_u32>,
    }


    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let device: stm32f4xx_hal::pac::Peripherals = ctx.device;
        let _syscfg = device.SYSCFG.constrain();
        // let reset_flags = device.RCC.csr.read();
        let rcc = device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(SYS_CK_MHZ.MHz())
            .require_pll48clk()
            .freeze();

        systick_init(&mut ctx.core.SYST, clocks);
        let gpioa = device.GPIOA.split();
        let gpioc = device.GPIOC.split();

        let mut delay = device.TIM5.delay_us(&clocks);
        let mut cs1 = gpioa.pa4.into_push_pull_output();
        let mut cs2 = gpioa.pa8.into_push_pull_output();
        let mut cs3 = gpioa.pa9.into_push_pull_output();
        let mut cs4 = gpioa.pa10.into_push_pull_output();
        let sck = gpioa.pa5.into_alternate();
        let miso = gpioa.pa6.into_alternate();
        let mosi = gpioa.pa7.into_alternate();
        let mut kyte1 = gpioc.pc0.into_push_pull_output();
        let mut kyte2 = gpioc.pc1.into_push_pull_output();
        let mut kyte3 = gpioc.pc2.into_push_pull_output();
        let mut peltier1 = gpioa.pa1.into_push_pull_output();
        let mut peltier2 = gpioa.pa2.into_push_pull_output();

        let mode = Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        };
        let mut spi = device.SPI1.spi((sck, miso, mosi), mode, 1_000_000.Hz(), &clocks);

        //kõik 4 termopaari kivi vaikimisi välja
        cs1.set_high();
        cs2.set_high();
        cs3.set_high();
        cs4.set_high();

        let usb = USB {
            usb_global: device.OTG_FS_GLOBAL,
            usb_device: device.OTG_FS_DEVICE,
            usb_pwrclk: device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

        let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

        let serial: usbd_serial::SerialPort<'static, UsbBus<USB>>;
        let usb_dev: UsbDevice<'static, UsbBus<USB>>;

        unsafe {
            USB_BUS = Some(usb_bus);
            serial = usbd_serial::SerialPort::new(USB_BUS.as_ref().unwrap());

            usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x05dc))
                //.self_powered(true)
                .manufacturer("Fake company")
                //.device_release(0x0010)
                .product("Serial port")
                .serial_number("TEST")
                .device_class(usbd_serial::USB_CLASS_CDC)
                .build();
        }

        (Shared { serial }, Local { usb_dev, spi, cs1, cs2, cs3, kyte1, kyte2, kyte3, peltier1, peltier2, delay },
         init::Monotonics())
    }


    #[idle(shared = [serial], local = [spi, cs1, cs2, cs3, kyte1, kyte2, kyte3, peltier1, peltier2, delay])]
    fn idle(mut ctx: idle::Context) -> ! {
        let mut next_time = get_time();
        let cs1 = ctx.local.cs1;
        let cs2 = ctx.local.cs2;
        let cs3 = ctx.local.cs3;
        //let cs4 = ctx.local.cs4;
        let spi = ctx.local.spi;
        let kyte1 = ctx.local.kyte1;
        let kyte2 = ctx.local.kyte2;
        let kyte3 = ctx.local.kyte3;
        let peltier1 = ctx.local.peltier1;
        let peltier2 = ctx.local.peltier2;
        let delay = ctx.local.delay;

        //pdm
        let mut pdm1 = Pdm::new(500000000, 0);
        let mut pdm2 = Pdm::new(1000000000, 0);
        pdm1.set_target(0.0);
        pdm2.set_target(0.0);
        kyte1.set_low();

        //PID variables
        let mut target_temp = 30.0; //setpoint
        let mut PID_error = 0.0;
        let mut previous_error = 0.0;
        let (mut elapsedTime,mut Time,mut timePrev): (f32,f32,f32);
        let mut PID_value = 0.0;
        let mut temperature_read = 0.0;
        let mut last_set_temperature = 0.0;

        //PID constants
        let (mut kp,mut ki,mut kd)=(90.0, 30.0, 80.0);
        let (mut PID_p,mut PID_i,mut PID_d)=(0.0, 0.0, 0.0);



        //peamine loop
        loop {
            let time = get_time();
            let mut termopaar_buf = ArrayString::<200>::new();
            let mut feedback_buf = ArrayString::<200>::new(); //testimiseks


            /* ctx.shared.serial.lock(|serial| {
                 match serial.read(buf.as_mut_slice()) {
                     Ok(count) if count > 0 => {
                         for c in buf[0..count].iter_mut() {
                             //write!(&mut feedback_buf, "{}\r\n", c);
                             target_temp = buf[c] as f32;
                         }

                     }
                     _ => {}
                 }
             });*/








            if time > next_time {
                //temp andur 1:
                match read_temperature(cs1, spi, delay) {
                    Ok(r) => {
                        write!(&mut termopaar_buf, "termopaar väline nr1: {}\r\n", r.temp);
                        temperature_read = r.temp as f64;
                    }
                    Err(e) => { write!(&mut termopaar_buf, "termopaar väline nr1: {:?}\r\n", e); }
                };

                PID_error = target_temp - temperature_read + 3.0; //arvutame vea sihttemp ja päris temp vahel
                PID_p = (0.01 * kp * PID_error) as f64;
                PID_i = 0.01*PID_i + (ki * PID_error);
                let PID_time = get_time();
                PID_d = 0.01*kd*((PID_error - previous_error)/(PID_time as f64/1000.0));
                PID_value = PID_p + PID_i + PID_d;

                //lisada: juhtimine

                previous_error = PID_error;

                next_time += 100000000;
            }
            /*
            //kontroll, kas saadud temperatuur on madalam soovitud temperatuurist. Kui on, siis läheb
            //küttekeha tööle
            if temperature_read < target_temp {
                if target_temp - temperature_read > 1.0 {
                    pdm1.set_target(1.0);
                } else { pdm1.set_target(0.0); }
                //kui praegune temp liiga suur, siis jahutame peltieriga
            } else if temperature_read >= target_temp {
                if temperature_read - target_temp > 1.0 {
                    pdm2.set_target(1.0);
                } else { pdm2.set_target(0.0); }

            }*/


            match pdm1.poll(time) {
                None => {}
                Some(true) => {
                    kyte1.set_high();
                }
                Some(false) => {
                    kyte1.set_low();
                }
            }
            match pdm2.poll(time) {
                None => {}
                Some(true) => {
                    kyte1.set_low();
                    peltier2.set_high();
                }
                Some(false) => {
                    peltier2.set_low();
                }
            }
            //write!(&mut feedback_buf, "termopaar väline nr1: {:?}\r\n", temp_feedback); //feedback testimiseks
            //write!(&mut feedback_buf, "temp target: {:?}\r\n", target_temp); //feedback testimiseks
            let mut test = [0u8, 4];
            let mut buf = ArrayString::<200>::new();
            write!(&mut buf, "PID value: {}\r\n", PID_value);

            ctx.shared.serial.lock(|serial| {
                serial.write(&mut termopaar_buf.as_bytes());
                serial.write(&mut buf.as_bytes());
            });

            /*ctx.shared.serial.lock(|serial| {
                let mut buf = [0u8; 64];
                serial.read(&mut buf);
            });*/
        }
    }

    ///funktsioon loeb termopaarilt saadud bittide jada ja tagastab termopaari
    /// temperatuuri C ning MAX31855 kivi sisemist temperatuuri.
    ///Samuti tagastab ka veakoodi, kui miskit on valesti.
    fn read_temperature(cs_pin: &mut impl OutputPin, mut spi: &mut impl Transfer<u8>, mut delay: &mut impl DelayUs<u32>) -> Result<FullResult, E> {
        cs_pin.set_low();
        delay.delay_us(10);
        let mut data = [0u8; 4];
        spi.transfer(&mut data[..]).ok();
        cs_pin.set_high();

        let x = u32::from_be_bytes(data);
        let fault = ((x & 0x10000) >> 16);
        let noConnection = (x & 0x1);

        let temp = (((x >> 16) as i16) >> 2) as f32 * 0.25;
        let inttemp = ((x as i16) >> 4) as f32 * 0.0625;

        //fault bit on 1 kui esineb viga. Edasi kontrollitakse, mis viga täpsemalt oli
        if fault == 1 {
            if noConnection == 1 {
                return Err(E::NoConnectionError);
            }
        }

        Ok(FullResult {
            temp,
            inttemp,
        })
    }

    #[task(binds = OTG_FS, local = [usb_dev], shared = [serial])]
    fn otg_fs_event(mut ctx: otg_fs_event::Context) {
        ctx.shared.serial.lock(|serial: &mut usbd_serial::SerialPort<'static, UsbBus<USB>>| {
            if !ctx.local.usb_dev.poll(&mut [serial]) {
                return;
            }
        });
    }

    #[task(binds = SysTick, priority = 15)]
    fn systick_tick(_: systick_tick::Context) {
        crate::TIME.fetch_add(1, Ordering::Relaxed);
    }
}

fn systick_init(syst: &mut SYST, clocks: Clocks) {
    let c_ck = clocks.sysclk().to_Hz();

    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((c_ck / 1000) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}

pub fn get_time() -> u64 {
    loop {
        let msec = TIME.load(Ordering::Relaxed);
        let syst = SYST::get_current();
        let msec2 = TIME.load(Ordering::Relaxed);
        if msec == msec2 {
            let x = (SYST_RELOAD - syst) * (1_000_000 / (SYST_RELOAD + 1));

            return (msec as u64) * 1_000_000 + x as u64;
        }
    }
}
