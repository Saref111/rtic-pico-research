#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use cortex_m::interrupt::Mutex;
    // use defmt::info;
    use rp_pico::hal;
    use rp_pico::hal::Clock;
    use cortex_m;
    use cortex_m::prelude::*;

   
    #[shared]
    struct Shared {
        channel: hal::pwm::Channel<hal::pwm::Pwm7, hal::pwm::FreeRunning, hal::pwm::B>,
        delay: cortex_m::delay::Delay,
    }

    #[local]
    struct Local {}

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
        

        let delay = cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().0);

        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let pwm_slices = hal::pwm::Slices::new(c.device.PWM, &mut resets);
        let mut pwm: hal::pwm::Slice<hal::pwm::Pwm7, hal::pwm::FreeRunning> = pwm_slices.pwm7;
        pwm.set_ph_correct();
        pwm.enable();

        let mut channel =  pwm.channel_b;
        channel.output_to(pins.gpio15);

        (
            Shared {
                channel: channel,
                delay: delay
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[idle(
        shared = [channel, delay]
    )]
    fn idle(cx: idle::Context) -> ! {
        let mut channel = cx.shared.channel;
        let mut delay = cx.shared.delay;

        
        loop {
            let cr = &mut channel;
            let dr = &mut delay;
            (cr, dr).lock(|c, d| {
                for i in (1..25_000).step_by(1000) {
                    defmt::info!("UP = {}", i);
                    c.set_duty(i);
                    d.delay_ms(100);
                }
                for i in (1..25_000).rev().step_by(1000) {
                    defmt::info!("UP = {}", i);
                    c.set_duty(i);
                    d.delay_ms(100);
                }
            });
            cortex_m::asm::nop();
        }
    }}
