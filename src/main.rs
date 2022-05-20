#![no_std]
#![no_main]
mod pdm;
mod pid;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m;
use cortex_m::peripheral::SYST;
use panic_halt as _;
use stm32f4xx_hal::rcc::Clocks;

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
    use stm32f4xx_hal::gpio::{Alternate, OpenDrain, Output};
    use stm32f4xx_hal::otg_fs::{USB, UsbBus};
    use stm32f4xx_hal::prelude::*;
    use stm32f4xx_hal::spi::{Mode, Phase, Polarity, Spi};
    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::prelude::*;
    use crate::pdm::Pdm;
    use crate::{get_time, SYS_CK_MHZ, systick_init};
    use crate::pid::PID;

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
        cs4: stm32f4xx_hal::gpio::Pin<'A', 10_u8, Output>,
        kyte1: stm32f4xx_hal::gpio::Pin<'C', 0_u8, Output>,
        kyte2: stm32f4xx_hal::gpio::Pin<'C', 1_u8, Output>,
        kyte3: stm32f4xx_hal::gpio::Pin<'C', 2_u8, Output>,
        peltier1: stm32f4xx_hal::gpio::Pin<'A', 1_u8, Output>,
        peltier2: stm32f4xx_hal::gpio::Pin<'A', 2_u8, Output>,
        delay: stm32f4xx_hal::timer::Delay<stm32f4xx_hal::pac::TIM5, 1000000_u32>,
        fan1: stm32f4xx_hal::gpio::Pin<'C', 8_u8, Output>,
        fan2: stm32f4xx_hal::gpio::Pin<'C', 9_u8, Output>,
        filter: stm32f4xx_hal::gpio::Pin<'B', 1_u8, Output>,
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
        let gpiob = device.GPIOB.split();

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
        let mut fan1=gpioc.pc8.into_push_pull_output();
        let mut fan2=gpioc.pc9.into_push_pull_output();
        let mut filter=gpiob.pb1.into_push_pull_output();

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

        (Shared { serial }, Local { usb_dev, spi, cs1, cs2, cs3, cs4, kyte1, kyte2, kyte3, peltier1, peltier2, fan1, fan2, filter,delay },
         init::Monotonics())
    }

    #[idle(shared = [serial], local = [spi, cs1, cs2, cs3, cs4, kyte1, kyte2, kyte3, peltier1, peltier2, fan1, fan2, filter, delay])]
    fn idle(mut ctx: idle::Context) -> ! {
        let mut next_time = get_time();
        let (cs1, cs2, cs3, cs4)=(ctx.local.cs1, ctx.local.cs2, ctx.local.cs3, ctx.local.cs4);
        let (kyte1, kyte2, kyte3)=(ctx.local.kyte1, ctx.local.kyte2, ctx.local.kyte3);
        let (peltier1, peltier2) = (ctx.local.peltier1, ctx.local.peltier2);
        let spi = ctx.local.spi;
        let delay = ctx.local.delay;
        let fan1=ctx.local.fan1;
        let filter=ctx.local.filter;
        let (mut kyte1_enable, mut kyte2_enable, mut peltier_enable, mut filter_enable)=(1, 1, 1, 0);

        //pdm
        let mut pdm1 = Pdm::new(1000000000, 0);
        let mut pdm2 = Pdm::new(1000000000, 0);
        let mut pdm3 = Pdm::new(1000000000, 0);
        pdm1.set_target(0.0);
        pdm2.set_target(0.0);
        pdm3.set_target(0.0);

        let (mut state1, mut state2, mut state3)=("","","");

        let mut pid1 = PID::new();
        let mut pid2 = PID::new();
        let mut pid3 = PID::new();
        //esimene katse: kp=0.06; kd=0.00001
        //teine katse: kp=0.1, kd=0.001
        //kolmas: kp=0,001 kd=0,0001
        pid1.kp = 0.08;
        pid1.kd=-0.00001;
        pid1.target_temp=0.0;

        pid2.kp = 0.06;
        pid2.kd=0.00001;
        pid2.target_temp=0.0;

        pid3.kp = -2.1;
        pid3.kd= -0.0000001;
        pid3.target_temp=24.0;

        fan1.set_low();
        //fan1.set_low();

        //peamine loop
        loop {
            let time = get_time();
            let mut buffer = ArrayString::<300>::new();
            let mut peltier_buf = ArrayString::<200>::new();

            if time > next_time {
                write!(&mut buffer, "time: {}; ", time / 1000000000);
                //temp andur 1 lugemine:
                if kyte1_enable==1 {
                    match read_temperature(cs1, spi, delay) {
                        Ok(r) => {
                            let mut output = pid1.set_input(r.temp);
                            pdm1.set_target(output);
                            write!(&mut buffer, "t1: {}, ", r.temp);
                            //write!(&mut termopaar_buf, "t1_pid_p: {}, ", pid1.pid_p);
                            //write!(&mut termopaar_buf, "t1_pid_d: {}, ", pid1.pid_d);
                            //write!(&mut termopaar_buf, "t1_state: {}, ", state1);
                            write!(&mut buffer, "t1_pid: {:.3}, ", output);
                            write!(&mut buffer, "t1_target: {};", pid1.target_temp);

                        }
                        Err(e) => { write!(&mut buffer, "t1: {:?}\r\n", e); }
                    };
                }
                //temp andur 2 lugemine:
                if kyte2_enable==1 {
                    match read_temperature(cs2, spi, delay) {
                        Ok(r) => {
                            let mut output = pid2.set_input(r.temp);
                            pdm2.set_target(output);
                            write!(&mut buffer, "t2: {}, ", r.temp);
                            //write!(&mut termopaar_buf, "t2_pid_p: {}, ", pid2.pid_p);
                            //write!(&mut termopaar_buf, "t2_pid_d: {}, ", pid2.pid_d);
                            //write!(&mut termopaar_buf, ", t2_state: {}, ", state2);
                            write!(&mut buffer, "t2_pid: {:.3}, ", output);
                            //write!(&mut termopaar_buf, "t2_time: {}, ", time/1000000000);
                            write!(&mut buffer, "t2_target: {};", pid2.target_temp);
                        }
                        Err(e) => { write!(&mut buffer, "t2: {:?}\r\n", e); }
                    };
                }
                if peltier_enable==1 {
                    match read_temperature(cs3, spi, delay) {
                        Ok(r) => {
                            let mut output = pid3.set_input(r.temp);
                            pdm3.set_target(output);
                            fan1.set_low();
                            write!(&mut buffer, "p1: {}, ", r.temp);
                            //write!(&mut buffer, ",p_state: {}, ", state3);
                            //write!(&mut buffer, "p_pid: {:.3};\r\n", output);
                        }
                        Err(e) => { write!(&mut buffer, "peltier1: {:?}\r\n", e); }
                    };
                }

                if filter_enable==1{
                    filter.set_low();
                    write!(&mut buffer, "filter: on; \r\n");
                }else {
                    filter.set_high();
                    //write!(&mut buffer, "filter: off; \r\n");
                }


                //parema loetavuse jaoks Puttys
                /*ctx.shared.serial.lock(|serial| {
                    serial.write(b"\r\n");
                    serial.write(b"\n");
                });*/
                next_time += 100000000;



            }

            match pdm1.poll(time) {
                None => {}
                Some(true) => {
                    state1 = " heating";
                    kyte1.set_high();
                }
                Some(false) => {
                    state1 = " cooling";
                    kyte1.set_low();
                }
            }
            match pdm2.poll(time) {
                None => {}
                Some(true) => {
                    state2 = " heating";
                    kyte2.set_high();
                }
                Some(false) => {
                    state2 = " cooling";
                    kyte2.set_low();
                }
            }
            match pdm3.poll(time) {
                None => {}
                Some(true) => {
                    state3=" on";
                    peltier1.set_high();
                }
                Some(false) => {
                    state3=" off";
                    peltier1.set_low();

                }
            }

            //andmete lugemine Serial pordist
            ctx.shared.serial.lock(|serial| {
                let mut buf = [0u8; 64];
                match serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        let tekst = core::str::from_utf8(&buf[..count]).unwrap();
                        let mut iter =tekst.split_whitespace();
                        //vastuvõetud andmete põhjal määratakse küttekehade soovitud temp

                        pid1.target_temp=iter.next().unwrap().parse().unwrap();
                        pid2.target_temp=iter.next().unwrap().parse().unwrap();
                        //pid3.target_temp=iter.next().unwrap().parse().unwrap();


                       /* let mut x:i32;
                        x=iter.next().unwrap().parse().unwrap();
                        if x==1 {
                            let mut temp:f32;
                            temp=iter.next().unwrap().parse().unwrap();

                            if temp==0.0 { kyte1_enable=0;}
                            else {
                                kyte1_enable=1;
                                pid1.target_temp= temp;
                            }

                        } else if x==2 {
                            let mut temp:f32;
                            temp=iter.next().unwrap().parse().unwrap();

                            if temp==0.0 { kyte2_enable=0;}
                            else {
                                kyte2_enable=1;
                                pid2.target_temp= temp;
                            }
                        } else if x==3 {
                            let mut temp:f32;
                            temp=iter.next().unwrap().parse().unwrap();

                            if temp==0.0 { peltier_enable=0;}
                            else {
                                peltier_enable=1;
                                pid3.target_temp= temp;
                            }
                        }*/
                    }
                    _ => ()
                };
            });
            ctx.shared.serial.lock(|serial| {
                serial.write(&mut buffer.as_bytes());
                //serial.write(&mut peltier_buf.as_bytes());
            });

            //andmete saatmine puhvrist üle Serial pordi

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
        let fault = (x & 0x10000) >> 16;
        let noConnection = x & 0x1;

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
