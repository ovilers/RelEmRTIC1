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


const WATCHDOG_TIMEOUT_MS: u64 = 20_000;

#[rtic::app(device = esp32c3, dispatchers = [FROM_CPU_INTR0, FROM_CPU_INTR1])]
mod app {

    use esp32c3_hal::{
        self as _,
        Rtc,
        clock::ClockControl,
        peripherals::{Peripherals, TIMG0, TIMG1},
        timer::{Timer, Timer0, TimerGroup, Wdt},
        prelude::*,
        IO, 
        gpio::{Input, PullUp, GpioPin, PushPull, Output}
    };

    use rtt_target::{rprintln, rtt_init_print};

    use crate::WATCHDOG_TIMEOUT_MS;

    #[shared]
    struct Shared {
        tempo: [u64;2]
    }

    #[local]
    struct Local {
        rtc: Rtc<'static>,
        watchdog: Wdt<TIMG1>,
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
        let timer_group1 = TimerGroup::new(
            peripherals.TIMG1,
            &clocks,
            &mut system.peripheral_clock_control,
        );
        

        let mut timer0 = timer_group0.timer0;
        let mut watchdog = timer_group1.wdt; 

        let tempo: u64 = 0;
        let rtc: Rtc<'_> = Rtc::new(peripherals.RTC_CNTL);

        let presstimes = [0, 0, 0];
        let pressindex = 0;

        let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
        
        let mut button = io.pins.gpio9.into_pull_up_input();
        let led = io.pins.gpio7.into_push_pull_output();

        watchdog.enable();
        watchdog.start(WATCHDOG_TIMEOUT_MS.millis());
        timer0.listen();
        timer0.start((100u64).millis());
        button.listen(esp32c3_hal::gpio::Event::FallingEdge);

        
        (
            Shared {
                tempo,
                
            },
            Local {
                rtc,
                watchdog,
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
          
        }
    }

    #[task(binds = TG0_T0_LEVEL, priority=3, local = [led, timer0], shared = [tempo])]
    fn timer0(cx: timer0::Context) {
        let timer0 = cx.local.timer0;
        let mut tempo = cx.shared.tempo;
        let led = cx.local.led;
        let mut toggle: bool = false;
        
        timer0.clear_interrupt();

        tempo.lock(|tempo| {
            // I wanted to keep locking as short as possible so we extract result of comparison in bool
            // and execute result outside of critical section
            if *tempo != 0{
                toggle = true;
                timer0.start((*tempo/2).millis());
            }
        });

        if toggle{ 
            led.toggle().unwrap_or_else(|e| rprintln!("Error toggling LED! {:?}", e));
        }
        else {
            timer0.start(100u64.millis());
        }
        

    }

    fn get_delta_u64(x1: u64, x2: u64) -> u64{
        if x1 > x2 {
            return x1 - x2;
        }
        return x2 - x1;
    }

    #[task(binds = GPIO, priority=2, local = [rtc, watchdog, button, presstimes, pressindex], shared = [tempo])]
    fn button(cx: button::Context) {
        rprintln!("button press");
        cx.local.watchdog.feed();
        let now = cx.local.rtc.get_time_ms();
        
        let pressindex = cx.local.pressindex;
        let presstimes = cx.local.presstimes;

        presstimes[*pressindex] = now;
        rprintln!("{}, {}, {}, {}", presstimes[0], presstimes[1], presstimes[2], pressindex);
        
        if *pressindex >= 2{
            *pressindex = 0;
        }
        else {
            *pressindex += 1;
        }

        // Led will only turn on after three presses
        let mut tempo = cx.shared.tempo;
        for i in 0..presstimes.len() {
            if presstimes[i] == 0{
                cx.local.button.clear_interrupt(); 
                return;
            }
        }
        let delta0;

        if presstimes[0] > presstimes[1]{
            delta0 = get_delta_u64(presstimes[0], presstimes[1]);        
        }
        else{
            delta0 = get_delta_u64(presstimes[0], presstimes[2]);
        }
        let delta1 = get_delta_u64(presstimes[1], presstimes[2]);

        tempo.lock(|tempo|{
            *tempo = (delta0 + delta1) / 2;
        });

        cx.local.button.clear_interrupt();
    }

}
