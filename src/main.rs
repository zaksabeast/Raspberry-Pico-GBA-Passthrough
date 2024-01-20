#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;
use fugit::RateExtU32;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::gpio::bank0::Gpio25;
use rp_pico::hal::gpio::{Output, Pin, PushPull};
use rp_pico::hal::{gpio, spi};
use rp_pico::hal::{pac, Clock};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
use usbd_webusb::*;

type LedPin = Pin<Gpio25, Output<PushPull>>;

fn blink(led_pin: &mut LedPin, delay: &mut Delay, count: usize) {
    for _ in 0..count {
        led_pin.set_high().unwrap();
        delay.delay_ms(300);
        led_pin.set_low().unwrap();
        delay.delay_ms(300);
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();
    blink(&mut led_pin, &mut delay, 2);

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut serial = SerialPort::new(&usb_bus);
    let mut wusb = WebUsb::new(&usb_bus, url_scheme::HTTP, "localhost:8000");
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1234, 0x2000))
        .product("GBA Passthrough")
        .serial_number("GBA")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let _spi_sclk = pins.gpio2.into_mode::<gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio3.into_mode::<gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<gpio::FunctionSpi>();

    let spi: spi::Spi<_, _, 8> = spi::Spi::new(pac.SPI0);
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        256_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    loop {
        if usb_dev.poll(&mut [&mut serial, &mut wusb]) {
            let mut buf = [0u8; 4];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(_count) => {
                    if let Ok(recv) = spi.transfer(&mut buf) {
                        let _ = serial.write(recv);
                    }
                }
            }
        }
    }
}

// End of file
