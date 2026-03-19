use defmt::*;
use embassy_stm32::gpio::{Level, Output};
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};

use crate::shared::RunState;

#[embassy_executor::task]
pub async fn led_task(
    mut led1: Output<'static>,
    mut led2: Output<'static>,
    state_watch: &'static Watch<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, RunState, 1>,
) {
    info!("led_task start");

    let mut rcv = state_watch.receiver().unwrap();

    // Initialize immediately (if a value is present); otherwise wait.
    let mut state = rcv.get().await;

    loop {
        match state {
            RunState::Running => {
                let mut ticker = Ticker::every(Duration::from_millis(100));
                loop {
                    led1.toggle();
                    led2.toggle();

                    // Wait either for time tick OR a state change.
                    // We do this by polling try_changed after each tick. It's simple and good enough
                    // for LED indication (not time-critical).
                    ticker.next().await;
                    if let Some(new_state) = rcv.try_changed() {
                        state = new_state;
                        break;
                    }
                }
            }
            RunState::Stopped => {
                let mut ticker = Ticker::every(Duration::from_millis(400));
                loop {
                    led1.set_level(Level::High);
                    led2.set_level(Level::Low);
                    ticker.next().await;
                    if let Some(new_state) = rcv.try_changed() {
                        state = new_state;
                        break;
                    }

                    led1.set_level(Level::Low);
                    led2.set_level(Level::High);
                    ticker.next().await;
                    if let Some(new_state) = rcv.try_changed() {
                        state = new_state;
                        break;
                    }
                }
            }
        }
    }
}
