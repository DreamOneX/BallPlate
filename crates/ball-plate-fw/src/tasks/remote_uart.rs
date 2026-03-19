use core::ffi::c_void;

use defmt::{info, warn};
use embassy_stm32::usart::BufferedUartRx;
use embassy_sync::channel::Channel;
use embedded_io_async::Read;

use ball_plate_protocol::command::{CmdId, StopMode};
use ball_plate_protocol::dispatcher::{CmdHandler, CommandDispatcher};
use ball_plate_protocol::framer::SerialFramer;
use ball_plate_protocol::packet::MAX_PACKET_SIZE;

use crate::shared::ControlRequest;

/// Remote control over UART (feature: remote-uart)
///
/// This task:
/// - continuously reads bytes from a buffered UART RX
/// - frames them into packets (A5..CRC)
/// - validates + deduplicates (handled by CommandDispatcher)
/// - pushes high-level [`ControlRequest`]s into a channel for `control_task`
#[cfg(feature = "remote-uart")]
#[embassy_executor::task]
pub async fn remote_uart_task(
    mut rx: BufferedUartRx<'static>,
    req_ch: &'static Channel<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, ControlRequest, 4>,
) {
    info!("remote_uart_task start");

    let req_snd = req_ch.sender();

    let mut dispatcher = CommandDispatcher::new();

    // Handler contexts are just pointers.
    // We'll store a DynamicSender inside the task stack and pass it by pointer.
    // Safe because dispatcher+ctx live for the whole task.
    let ctx = Ctx {
        req_snd: req_snd.into(),
    };
    let ctx_ptr: *mut c_void = (&ctx as *const Ctx).cast::<c_void>() as *mut c_void;

    dispatcher.on(CmdId::REMOTE_STOP, on_stop as CmdHandler, ctx_ptr);
    dispatcher.on(CmdId::REMOTE_RESUME, on_resume as CmdHandler, ctx_ptr);
    dispatcher.on(CmdId::TARGET_UPDATE, on_target_update as CmdHandler, ctx_ptr);

    let mut framer = SerialFramer::new();
    let mut pkt_buf = [0u8; MAX_PACKET_SIZE];

    let mut b = [0u8; 1];
    loop {
        let n = match rx.read(&mut b).await {
            Ok(n) => n,
            Err(e) => {
                warn!("remote uart read err: {:?}", e);
                continue;
            }
        };
        if n == 0 {
            continue;
        }

        if let Some(pkt_len) = framer.feed(b[0], &mut pkt_buf) {
            // `CommandDispatcher` expects a packet-oriented receiver.
            // We already have exactly one packet, so adapt it.
            struct OneShotRx {
                buf: [u8; MAX_PACKET_SIZE],
                len: usize,
                used: bool,
            }
            impl ball_plate_protocol::dispatcher::CommandReceiver for OneShotRx {
                fn receive(&mut self, out: &mut [u8; MAX_PACKET_SIZE]) -> Option<usize> {
                    if self.used {
                        return None;
                    }
                    out[..self.len].copy_from_slice(&self.buf[..self.len]);
                    self.used = true;
                    Some(self.len)
                }
            }

            let mut oneshot = OneShotRx {
                buf: pkt_buf,
                len: pkt_len,
                used: false,
            };
            dispatcher.poll(&mut oneshot);

            // Refresh local packet buffer for next frame.
            pkt_buf = oneshot.buf;
        }
    }
}

#[cfg(feature = "remote-uart")]
fn on_stop(ctx: *mut c_void, payload: *const u8, len: usize) {
    // payload: [mode]
    if len < 1 {
        return;
    }
    let mode = unsafe { *payload };
    let mode = StopMode::try_from(mode).unwrap_or(StopMode::Freeze);

    let ctx = unsafe { &*(ctx as *const Ctx) };
    let _ = ctx.req_snd.try_send(ControlRequest::Stop(mode));
}

#[cfg(feature = "remote-uart")]
fn on_resume(ctx: *mut c_void, _payload: *const u8, _len: usize) {
    let ctx = unsafe { &*(ctx as *const Ctx) };
    let _ = ctx.req_snd.try_send(ControlRequest::Resume);
}

#[cfg(feature = "remote-uart")]
fn on_target_update(ctx: *mut c_void, payload: *const u8, len: usize) {
    // payload: f32 x, f32 y (little endian)
    if len < 8 {
        return;
    }
    let p = unsafe { core::slice::from_raw_parts(payload, len) };
    let x = f32::from_le_bytes([p[0], p[1], p[2], p[3]]);
    let y = f32::from_le_bytes([p[4], p[5], p[6], p[7]]);

    let ctx = unsafe { &*(ctx as *const Ctx) };
    let _ = ctx.req_snd.try_send(ControlRequest::TargetUpdate { x, y });
}

#[cfg(feature = "remote-uart")]
struct Ctx {
    req_snd: embassy_sync::channel::DynamicSender<'static, ControlRequest>,
}
