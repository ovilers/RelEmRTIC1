//! tap_tempo
//!
//! Run on target: `cd esp32c3`
//!
//! cargo embed --example tap_tempo --release
//! 
//! Blinks led in tempo of button presses
//!
//!
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// bring in panic handler
use panic_rtt_target as _;

#[rtic::app(device = esp32c3, dispatchers = [FROM_CPU_INTR0, FROM_CPU_INTR1])]
mod app {

    use esp32c3_hal::{
        self as _,
        Rtc,
        clock::ClockControl,
        peripherals::{Peripherals, TIMG0},
        timer::{Timer, Timer0, TimerGroup},
        prelude::*,
        IO, 
        gpio::{Input, PullUp, GpioPin, PushPull, Output}, riscv::asm::wfi,
    };

    use rtt_target::{rprintln, rtt_init_print};


    #[shared]
    struct Shared {
        tempo: u64
    }

    #[local]
    struct Local {
        rtc: Rtc<'static>,
        presstimes: [u64;3],
        pressindex: usize,
        button: GpioPin<Input<PullUp>, 9>,
        led: GpioPin<Output<PushPull>, 7>,
        timer0: Timer<Timer0<TIMG0>>
    }

    #[init]
    fn init(_: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        rprintln!("tap_tempo");

        

        let peripherals = Peripherals::take();
        let mut system = peripherals.SYSTEM.split();
        let clocks = ClockControl::max(system.clock_control).freeze();

        let timer_group0 = TimerGroup::new(
            peripherals.TIMG0,
            &clocks,
            &mut system.peripheral_clock_control,
        );
        let mut timer0 = timer_group0.timer0;

        let tempo: u64 = 1000;
        let rtc: Rtc<'_> = Rtc::new(peripherals.RTC_CNTL);

        let mut presstimes = [0, 1, 2];
        let mut pressindex = 0;

        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
        
        let mut button = io.pins.gpio9.into_pull_up_input();

        let led = io.pins.gpio7.into_push_pull_output();

        timer0.listen();
        timer0.start((tempo/2).millis());
        button.listen(esp32c3_hal::gpio::Event::FallingEdge);

        
        (
            Shared {
                tempo,
                
            },
            Local {
                rtc,
                presstimes,
                pressindex,
                button,
                led,
                timer0
            },
        )
    }

    #[idle(local = [ ])]
    fn idle(_: idle::Context) -> ! {
        loop {
            // Enable this for power saving feature
            /*
            unsafe{
                wfi();
            };
            */
        }
    }

    #[task(binds = TG0_T0_LEVEL, priority=3, local = [led, timer0], shared = [tempo])]
    fn timer0(cx: timer0::Context) {
        let timer0 = cx.local.timer0;
        let mut tempo = cx.shared.tempo;
        let led = cx.local.led;
        led.toggle().unwrap();

        timer0.clear_interrupt();
        tempo.lock(|tempo| {
            timer0.start((*tempo/2).millis());
        });
        

    }

    #[task(binds = GPIO, priority=2, local = [rtc, button, presstimes, pressindex], shared = [tempo])]
    fn button(cx: button::Context) {
        rprintln!("button press");
        let now = cx.local.rtc.get_time_ms();
        
        let pressindex = cx.local.pressindex;
        let presstimes = cx.local.presstimes;

        // Prevent overly long tempos caused by an unfinished sequence affecting later sequence 
        if *pressindex > 0 && (now - presstimes[*pressindex-1]) > 10000 {
            *pressindex = 0
        }

        presstimes[*pressindex] = now;
        rprintln!("{}, {}, {}, {}", presstimes[0], presstimes[1], presstimes[2], pressindex);
        
        if *pressindex >= 2{
            
            let mut tempo = cx.shared.tempo;
            tempo.lock(|tempo|{
                *tempo = (presstimes[2] - presstimes[1] + presstimes[1] - presstimes[0]) / 2;
            });
            *pressindex = 0;
        }
        else {
            *pressindex += 1;
        }
        
        cx.local.button.clear_interrupt();
    }

}
