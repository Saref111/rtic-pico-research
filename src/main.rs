#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use hal::gpio::{DynPin, DYN_PUSH_PULL_OUTPUT};
    use rp_pico::hal;
    use rp_pico::hal::Clock;
    use cortex_m;
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
            pins.gpio15.into(),
            pins.gpio14.into(),
            pins.gpio13.into(),
            pins.gpio12.into(),
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
        let pins = cx.local.a;

        for pin in pins.iter_mut() {
            pin.into_push_pull_output();
            pin.try_into_mode(DYN_PUSH_PULL_OUTPUT).unwrap();
            pin.set_high().unwrap();
        }
        loop {
            cortex_m::asm::nop();
        }
    }
}
