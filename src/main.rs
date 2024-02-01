#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    extern crate alloc;

    use hal::gpio::DynPin;
    use rp_pico::hal;
    use rp_pico::hal::gpio::pin::Pin;
    use rp_pico::hal::Clock;
    use cortex_m;
    use core::convert::TryInto;
    use embedded_hal::digital::v2::OutputPin;

    type PinsArray = [DynPin; 4];

    #[shared]
    struct Shared {
    }
    
    #[local]
    struct Local {
        a: PinsArray
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {

        let mut resets = c.device.RESETS;
        let mut watchdog = hal::watchdog::Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog
        ).ok().unwrap();
        

        let _delay = cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().0);

        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let pinarray: PinsArray = [
            pins.gpio2.into(),
            pins.gpio3.into(),
            pins.gpio4.into(),
            pins.gpio5.into(),
        ];


        (
            Shared {
            },
            Local {
                
                a: pinarray
            },
            init::Monotonics(),
        )
    }

    #[idle(
        local = [a]
    )]
    fn idle(cx: idle::Context) -> ! {
        let mut pins = cx.local.a;

        for (i, pin) in pins.iter_mut().enumerate() {
            match i {
                0 => {
                    pin.into_push_pull_output();
                    let mut pin: Pin<hal::gpio::bank0::Gpio0, hal::gpio::PushPullOutput> = (*pin).try_into().unwrap();
                    OutputPin::set_high(&mut pin).expect("0 pin is not set high");
                },
                1 => {
                    pin.into_push_pull_output();
                    let mut pin: Pin<hal::gpio::bank0::Gpio1, hal::gpio::PushPullOutput> = (*pin).try_into().unwrap();
                    OutputPin::set_high(&mut pin).unwrap();
                }
                2 => {
                    pin.into_push_pull_output();
                    let mut pin: Pin<hal::gpio::bank0::Gpio2, hal::gpio::PushPullOutput> = (*pin).try_into().unwrap();
                    OutputPin::set_high(&mut pin).unwrap();
                }
                4 => {
                    pin.into_push_pull_output();
                    let mut pin: Pin<hal::gpio::bank0::Gpio3, hal::gpio::PushPullOutput> = (*pin).try_into().unwrap();
                    OutputPin::set_high(&mut pin).unwrap();
                }
                _ => break
            }
        }
        loop {
            cortex_m::asm::nop();
        }
    }
}
