use defmt::{info, warn};
use embassy_stm32::usart::BufferedUartTx;
use embassy_sync::watch::Watch;
use embedded_io_async::Write;

use crate::shared::ControlOutput;

/// Telemetry reporting task (feature: telemetry)
///
/// Matches the C++ CSV-ish lines from `reportToHost()`:
///   X:curr,target,error,output,angle
///   Y:curr,target,error,output,angle
///
/// Note: at the moment we only have `ControlOutput` (pos + angles) in fw.
/// PID internals (target/error/output) aren't yet plumbed through.
#[cfg(feature = "telemetry")]
#[embassy_executor::task]
pub async fn telemetry_task(
    mut tx: BufferedUartTx<'static>,
    out_watch: &'static Watch<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, ControlOutput, 1>,
) {
    info!("telemetry_task start");

    let mut rcv = out_watch.receiver().unwrap();

    loop {
        let out = rcv.changed().await;

        // Keep it lightweight: fixed-format, no heap.
        // NOTE: defmt doesn't write to UART; this is real UART output.
        // We use core::fmt into a stack buffer.
        let mut buf = [0u8; 128];
        let n = format_line(&mut buf, out);

        if let Err(e) = tx.write(&buf[..n]).await {
            warn!("telemetry uart err: {:?}", e);
        }
    }
}

#[cfg(feature = "telemetry")]
fn format_line(buf: &mut [u8], out: ControlOutput) -> usize {
    use core::fmt::Write as _;

    struct W<'a> {
        b: &'a mut [u8],
        n: usize,
    }

    impl core::fmt::Write for W<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            let rem = self.b.len().saturating_sub(self.n);
            let take = bytes.len().min(rem);
            self.b[self.n..self.n + take].copy_from_slice(&bytes[..take]);
            self.n += take;
            Ok(())
        }
    }

    let mut w = W { b: buf, n: 0 };

    // For now: output both axes on one line.
    // TODO(telemetry): match exact C++ host_report formatting (two lines, include PID details).
    let _ = core::write!(
        &mut w,
        "POS:{:.2},{:.2} ANG:{:.2},{:.2}\n",
        out.pos.x,
        out.pos.y,
        out.angle_x,
        out.angle_y
    );

    w.n
}
