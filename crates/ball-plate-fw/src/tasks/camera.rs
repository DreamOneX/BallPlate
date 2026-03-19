use defmt::*;
use embassy_stm32::usart::RingBufferedUartRx;
use embassy_sync::watch::Watch;

use crate::shared::Position;

// Match C++ line buffer size (uart_pos_provider.hpp)
const LINE_BUF_LEN: usize = 32;

#[embassy_executor::task]
pub async fn camera_task(
    mut rx: RingBufferedUartRx<'static>,
    pos_watch: &'static Watch<embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, Position, 2>,
) {
    info!("camera_task start");

    let snd = pos_watch.sender();

    // Small DMA→parser staging buffer. RingBufferedUartRx::read() returns as soon as
    // data is available.
    let mut buf = [0u8; 64];

    let mut line = [0u8; LINE_BUF_LEN];
    let mut len: usize = 0;

    loop {
        let n = match rx.read(&mut buf).await {
            Ok(n) => n,
            Err(e) => {
                warn!("camera uart err: {:?}", e);
                continue;
            }
        };

        for &b in &buf[..n] {
            if b == b'\n' {
                if let Some(pos) = parse_line(&line[..len]) {
                    snd.send(pos);
                }

                len = 0;
                continue;
            }

            // Copy byte if there's space; else drop until newline (same spirit as C++: it truncates)
            if len < LINE_BUF_LEN - 1 {
                line[len] = b;
                len += 1;
            }
        }
    }
}

fn parse_line(bytes: &[u8]) -> Option<Position> {
    // Expect ASCII like: "123.4,56.7" (no \n)
    let comma = bytes.iter().position(|&b| b == b',')?;

    let (x_bytes, y_bytes) = bytes.split_at(comma);
    let y_bytes = &y_bytes[1..];

    let x = parse_f32(x_bytes)?;
    let y = parse_f32(y_bytes)?;

    Some(Position { x, y })
}

fn parse_f32(bytes: &[u8]) -> Option<f32> {
    // Trim spaces and \r.
    let bytes = trim_ascii(bytes);
    if bytes.is_empty() {
        return None;
    }

    let s = core::str::from_utf8(bytes).ok()?;

    // fast-float parsing isn't in core. We'll rely on Rust's float parser.
    // (On embedded this pulls in some code, but acceptable for now.)
    s.parse::<f32>().ok()
}

fn trim_ascii(mut s: &[u8]) -> &[u8] {
    while let Some((&b, rest)) = s.split_first() {
        if b == b' ' || b == b'\r' || b == b'\t' {
            s = rest;
        } else {
            break;
        }
    }
    while let Some((&b, rest)) = s.split_last() {
        if b == b' ' || b == b'\r' || b == b'\t' {
            s = rest;
        } else {
            break;
        }
    }
    s
}
