
#![allow(clippy::empty_loop)]
#![no_std]
#![no_main]



use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::{pac, prelude::*};
use usb_device::prelude::*;
use core::fmt::Write;
use arrayvec::ArrayString;
use stm32f4xx_hal::spi::{Mode, Phase, Polarity};


static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .require_pll48clk()
        .freeze();

    let gpioa = dp.GPIOA.split();

    let mut cs1 = gpioa.pa4.into_push_pull_output();
    let mut cs2 = gpioa.pa8.into_push_pull_output();
    let mut cs3 = gpioa.pa9.into_push_pull_output();
    let mut cs4 = gpioa.pa10.into_push_pull_output();
    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();

    let mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnSecondTransition,
    };

    let mut delay = dp.TIM5.delay_us(&clocks);
    let mut spi = dp.SPI1.spi((sck, miso, mosi), mode, 1_000_000.Hz(), &clocks);

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        //.self_powered(true)
        .manufacturer("Fake company")
        //.device_release(0x0010)
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let mut data = [0;4];
    cs1.set_high();
    cs2.set_high();
    cs3.set_high();
    cs4.set_high();

    loop {


        cs1.set_high();
        delay.delay_ms(1_u32);
        cs1.set_low();
        spi.transfer(&mut data[..]).unwrap();
        // delay.delay_ms(100_u32);
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        // 200 baidine buffer
        let mut buf = ArrayString::<200>::new();

        for x in data {
            write!(&mut buf, "x= {}\r\n",x).ok();
        }
            // Formaatimist vt siit https://doc.rust-lang.org/std/fmt/

        //}
        //serial.write(&data[0..4]);
        serial.write(buf.as_bytes());
        delay.delay_ms(500_u32);


    }
}