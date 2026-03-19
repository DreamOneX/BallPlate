use defmt::*;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};

use ball_plate_pid::PidController;

use crate::config;
use crate::drivers::pca9685::Pca9685;
use crate::shared::{ControlOutput, ControlRequest, Position, RunState, StopMode};

#[embassy_executor::task]
pub async fn control_task(
    mut pca: Pca9685<'static>,
    pos_watch: &'static Watch<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, Position, 2>,
    state_watch: &'static Watch<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, RunState, 1>,
    out_watch: &'static Watch<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, ControlOutput, 1>,
    req_ch: &'static Channel<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, ControlRequest, 4>,
) {
    info!("control_task start");

    let mut pos_rcv = pos_watch.receiver().unwrap();

    let state_snd = state_watch.sender();
    let out_snd = out_watch.sender();

    let req_rcv = req_ch.receiver();

    // Start running by default.
    let mut stopped = false;
    let mut stop_mode = StopMode::Freeze;
    let mut park_written = false;

    let mut pending_resume = false;
    let mut pending_target: Option<(f32, f32)> = None;

    let mut pid_x = PidController::new(config::KP, config::KI, config::KD, config::CENTER_X);
    let mut pid_y = PidController::new(config::KP, config::KI, config::KD, config::CENTER_Y);

    // Center servos at boot (parity with C++ setup())
    // (Ignore errors for now; if I2C is not wired, we'll just keep running.)
    let _ = write_servo_deg(
        &mut pca,
        config::SERVO_X_CHANNEL,
        config::SERVO_CENTER_DEG + config::SERVO_X_OFFSET_DEG,
    )
    .await;
    let _ = write_servo_deg(
        &mut pca,
        config::SERVO_Y_CHANNEL,
        config::SERVO_CENTER_DEG + config::SERVO_Y_OFFSET_DEG,
    )
    .await;

    let mut ticker = Ticker::every(Duration::from_hz(config::CONTROL_FREQ_HZ as u64));
    let dt = 1.0 / (config::CONTROL_FREQ_HZ as f32);

    loop {
        ticker.next().await;

        // Drain remote/control requests (non-blocking).
        while let Ok(req) = req_rcv.try_receive() {
            match req {
                ControlRequest::Stop(mode) => {
                    stopped = true;
                    stop_mode = mode;
                    park_written = false;
                    pending_resume = false;

                    state_snd.send(RunState::Stopped);
                }
                ControlRequest::Resume => {
                    // Mirror C++: resume is "pending" and PID reset is done in control context.
                    pending_resume = true;
                }
                ControlRequest::TargetUpdate { x, y } => {
                    // Mirror C++ semantics: retain target updates even while stopped.
                    pending_target = Some((x, y));
                }
            }
        }

        // 1) Consume pending resume (checked before stop gate)
        if pending_resume {
            pid_x.reset();
            pid_y.reset();
            stopped = false;
            pending_resume = false;
            park_written = false;

            state_snd.send(RunState::Running);
        }

        // 2) Stop gate
        if stopped {
            if stop_mode == StopMode::Park && !park_written {
                let _ = write_servo_deg(
                    &mut pca,
                    config::SERVO_X_CHANNEL,
                    config::SERVO_CENTER_DEG + config::SERVO_X_OFFSET_DEG,
                )
                .await;
                let _ = write_servo_deg(
                    &mut pca,
                    config::SERVO_Y_CHANNEL,
                    config::SERVO_CENTER_DEG + config::SERVO_Y_OFFSET_DEG,
                )
                .await;
                park_written = true;
            }
            continue;
        }

        // 3) Consume pending target update
        if let Some((x, y)) = pending_target.take() {
            pid_x.set_target(x);
            pid_y.set_target(y);
        }

        // 4) Consume latest camera position (only act when we got a fresh update)
        let pos = match pos_rcv.try_changed() {
            Some(p) => p,
            None => continue,
        };

        let angle_x = config::SERVO_CENTER_DEG + pid_x.compute(pos.x, dt);
        let angle_y = config::SERVO_CENTER_DEG + pid_y.compute(pos.y, dt);

        // Apply outputs (I2C write is allowed here, unlike the C++ ISR)
        let _ = write_servo_deg(
            &mut pca,
            config::SERVO_X_CHANNEL,
            angle_x + config::SERVO_X_OFFSET_DEG,
        )
        .await;
        let _ = write_servo_deg(
            &mut pca,
            config::SERVO_Y_CHANNEL,
            angle_y + config::SERVO_Y_OFFSET_DEG,
        )
        .await;

        out_snd.send(ControlOutput {
            pos,
            angle_x,
            angle_y,
        });
    }
}

async fn write_servo_deg(
    pca: &mut Pca9685<'static>,
    channel: u8,
    angle_deg: f32,
) -> Result<(), embassy_stm32::i2c::Error> {
    let effective = angle_deg.clamp(0.0, 180.0);
    let span = (config::SERVO_MAX_PULSE_US - config::SERVO_MIN_PULSE_US) as f32;
    let pulse = (config::SERVO_MIN_PULSE_US as f32) + (effective / 180.0) * span;

    // `f32::round()` isn't always available in `core` depending on build settings,
    // and `pulse` is always positive here.
    let pulse = (pulse + 0.5) as u16;

    pca.write_microseconds(channel, pulse).await
}
