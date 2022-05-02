#![no_main]
#![no_std]

extern crate panic_halt;
use panic_halt as _;

use core::fmt::Write;
use core::cell::{Cell, RefCell};
use core::ops::{DerefMut, Not};
use arrayvec::ArrayString;
use cortex_m;
use cortex_m::interrupt::{free, CriticalSection, Mutex};
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use stm32f4xx_hal::i2c::Error::Timeout;
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::{prelude::*};
use stm32f4xx_hal::spi::{Mode, Phase, Polarity, Spi};
use stm32f4xx_hal::time::Hertz;
use usb_device::prelude::*;

use crate::hal::{
    gpio::{Edge, Input, PC13},
    i2c::I2c,
    interrupt, pac,
    prelude::*,
    rcc::{Clocks, Rcc},
    timer::{CounterUs, Event, Timer},
};

static mut EP_MEMORY: [u32; 1024] = [0; 1024];
static TIMER_TIM2: Mutex<RefCell<Option<CounterUs<pac::TIM2>>>> = Mutex::new(RefCell::new(None));
static USB_STATE: Mutex<Cell<bool>> = Mutex::new(Cell::new(false));


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();

    // interrupti näite GPIO
    let gpiob = dp.GPIOB.split();
    let mut led = gpiob.pb8.into_push_pull_output();

    // Set up the system clock
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(48.MHz())
        .require_pll48clk()
        .freeze();

    let mut delay = dp.TIM5.delay_us(&clocks);

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
        phase: Phase::CaptureOnFirstTransition,
    };
    let mut spi = dp.SPI1.spi((sck, miso, mosi), mode, 1_000_000.Hz(), &clocks);

    cs1.set_high();
    cs2.set_high();
    cs3.set_high();
    cs4.set_high();

    //USB init

        //väike pauss, simuleerib USB restarti.
    let mut usb_pin_d_plus = gpioa.pa12.into_push_pull_output();
    usb_pin_d_plus.set_low();
    delay.delay_ms(100_u32);
        // Now we can connect as a USB serial device to the host.
    let usb_pin_d_plus = usb_pin_d_plus.into_alternate();
    let usb_pin_d_minus = gpioa.pa11.into_alternate();

    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: usb_pin_d_minus,
        pin_dp: usb_pin_d_plus,
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

    // Set up the interrupt timer
    let mut timer = dp.TIM2.counter(&clocks);
    timer.start(1.secs()).unwrap();
    timer.listen(Event::Update);

    free(|cs| {
        TIMER_TIM2.borrow(cs).replace(Some(timer));
    });

    // Enable interrupt
    pac::NVIC::unpend(hal::pac::Interrupt::TIM2);
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::TIM2);
    }

    let mut data = [0;4];

    loop {

        //termopaari kivi lugemine
        cs1.set_high();
        delay.delay_ms(1_u32);
        cs1.set_low();
        spi.transfer(&mut data[..]).unwrap();

        // 200 baidine buffer
        let mut buf = ArrayString::<200>::new();
        for x in data {
            write!(&mut buf, "x= {}\r\n",x).ok();
        }

        if free(|cs| USB_STATE.borrow(cs).get()) {
            serial.write(buf.as_bytes());
        } else {
            continue;
            //led.set_low();
        }
        //usb_dev.poll(&mut [&mut serial]);
    }

    #[interrupt]
    fn TIM2() {
        free(|cs| {
            if let Some(ref mut tim2) = TIMER_TIM2.borrow(cs).borrow_mut().deref_mut() {
                tim2.clear_interrupt(Event::Update);
            }


            //usb_dev.poll(&mut [&mut serial]);

            //let led_state = LED_STATE.borrow(cs);
            //led_state.replace(led_state.get().not());
        });
    }
}