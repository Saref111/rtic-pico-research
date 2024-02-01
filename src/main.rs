#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    extern crate alloc;

    use core::convert::Infallible;

    use alloc::boxed::Box;
    use hal::gpio::bank0::Gpio0;
    use hal::gpio::{DynPin, Input, Pin, PinId, PinMode, PullDown, PushPullOutput, DYN_PUSH_PULL_OUTPUT};
    // use alloc::{vec, vec::Vec};
    // use cortex_m::interrupt::Mutex;
    // use hal::gpio::{DynPinId, Function, PushPullOutput};
    // use defmt::info;
    use rp_pico::hal;
    use rp_pico::hal::Clock;
    use cortex_m;
    use core::convert::TryInto;

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
        // let p = pins.gpio13.into_pull_up_input();
        let pinarray: PinsArray = [
            pins.gpio2.into(),
            pins.gpio3.into(),
            pins.gpio4.into(),
            pins.gpio5.into(),
        ];

    
        // ... and so on for each pin you want to add
        // let pwm_slices = hal::pwm::Slices::new(c.device.PWM, &mut resets);
        // let mut pwm: hal::pwm::Slice<hal::pwm::Pwm7, hal::pwm::FreeRunning> = pwm_slices.pwm7;
        // pwm.set_ph_correct();
        // pwm.enable();

        // let mut channel =  pwm.channel_b;
        // channel.output_to(pins.gpio15);

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
                    let pin: Pin<hal::gpio::bank0::Gpio0, hal::gpio::PushPullOutput> = (*pin).try_into().unwrap();
                    // pin
                },
                1 => {
                    pin.into_push_pull_output();
                    let gpio0: Pin<hal::gpio::bank0::Gpio1, hal::gpio::PushPullOutput> = (*pin).try_into().unwrap();
                }
                2 => {
                    pin.into_push_pull_output();
                    let gpio0: Pin<hal::gpio::bank0::Gpio1, hal::gpio::PushPullOutput> = (*pin).try_into().unwrap();
                }
                _ => break
            }
            // let id = pin.id();
            // id.num
        }
        loop {
            cortex_m::asm::nop();
        }
    }
}
