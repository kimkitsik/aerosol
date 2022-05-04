#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m;
use cortex_m::peripheral::SYST;
use max31855::Error;
use panic_halt as _;
use stm32f4xx_hal::rcc::Clocks;
use max31855::Max31855;

pub static TIME: AtomicU32 = AtomicU32::new(0);
pub const SYS_CK_MHZ: u32 = 144;
pub const SYST_RELOAD: u32 = (SYS_CK_MHZ * 1000) - 1;

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use core::fmt::Write;
    use core::sync::atomic::Ordering;

    use arrayvec::ArrayString;
    use max31855::Unit;
    use stm32f4xx_hal::gpio::{Alternate, Output};
    use stm32f4xx_hal::otg_fs::{USB, UsbBus};
    use stm32f4xx_hal::prelude::*;
    use stm32f4xx_hal::spi::{Mode, Phase, Polarity, Spi};
    use systick_monotonic::fugit::Duration;
    use systick_monotonic::Systick;
    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::prelude::*;

    use crate::{get_time, SYS_CK_MHZ, systick_init};

    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

    #[shared]
    struct Shared {
        serial: usbd_serial::SerialPort<'static, UsbBus<USB>>,
    }

    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        spi: Spi<stm32f4xx_hal::pac::SPI1, (stm32f4xx_hal::gpio::Pin<'A', 5_u8, Alternate<5_u8>>, stm32f4xx_hal::gpio::Pin<'A', 6_u8, Alternate<5_u8>>, stm32f4xx_hal::gpio::Pin<'A', 7_u8, Alternate<5_u8>>)>,
        cs1: stm32f4xx_hal::gpio::Pin<'A', 4_u8, Output>,
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

        let mut delay = device.TIM5.delay_us(&clocks);
        let mut cs1 = gpioa.pa4.into_push_pull_output();
        let mut cs2 = gpioa.pa8.into_push_pull_output();
        let mut cs3 = gpioa.pa9.into_push_pull_output();
        let mut cs4 = gpioa.pa10.into_push_pull_output();
        let sck = gpioa.pa5.into_alternate();
        let miso = gpioa.pa6.into_alternate();
        let mosi = gpioa.pa7.into_alternate();

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

        (Shared { serial }, Local { usb_dev, spi, cs1, delay }, init::Monotonics())
    }

    #[idle(shared = [serial])] //, local = [spi, cs1, delay]
    fn idle(mut ctx: idle::Context) -> ! {
        let mut next_time = get_time();

        loop {
            let time = get_time();

            if time > next_time {
                //termopaari lugemine


                read_thermo();
                //read_thermo::spawn_after().unwrap();

                next_time += 1000000000;
            }

            ctx.shared.serial.lock(|serial| {
                let mut buf = [0u8; 64];
                serial.read(&mut buf);
            });
        }
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

    #[task(local=[cs1,delay,spi])]
    fn read_thermo(_: read_thermo::Context){
        let mut buf = ArrayString::<200>::new();
        let mut data = [0u8; 4];

        ctx.local.cs1.set_high();
        ctx.local.delay.delay_ms(1_u32);
        ctx.local.cs1.set_low();
        ctx.local.delay.delay_ms(1_u32);

        ctx.local.spi.transfer(&mut data[..]).unwrap();
        let x = u32::from_be_bytes(data);

        let temp = (((x >> 16) as i16) / 4) as f32 * 0.25;

        write!(&mut buf, "{}\r\n", temp);
        ctx.shared.serial.lock(|serial| {
            serial.write(&mut buf.as_bytes()); //.as_bytes()
            //serial.write(temp);
        });

        //read_thermo::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).ok();
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