//! Serial byte-stream framer (SYNC → HEADER → PAYLOAD+CRC).

#![allow(dead_code)]

use crate::packet::{parse_packet, HEADER_SIZE, MAX_PACKET_SIZE, MAX_PAYLOAD, OFF_LEN, PACKET_HEADER};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Sync,
    Header,
    PayloadCrc,
}

/// Frames a byte stream into complete validated packets.
///
/// Transport-agnostic port of `SerialReceiver`'s state machine. Unlike the packet-oriented
/// nRF24L01 link, serial is a byte stream and must be framed.
///
/// When a full valid packet is received, bytes are copied into the caller-provided output buffer
/// and the total length is returned.
pub struct SerialFramer {
    buf: [u8; MAX_PACKET_SIZE],
    pos: usize,
    expected_total: usize,
    state: State,
}

impl SerialFramer {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_PACKET_SIZE],
            pos: 0,
            expected_total: 0,
            state: State::Sync,
        }
    }

    pub fn reset(&mut self) {
        self.state = State::Sync;
        self.pos = 0;
        self.expected_total = 0;
    }

    /// Feed one byte from the stream.
    ///
    /// Returns `Some(len)` when a complete valid packet has been framed; the packet bytes are
    /// copied into `out[..len]`.
    pub fn feed(&mut self, byte: u8, out: &mut [u8; MAX_PACKET_SIZE]) -> Option<usize> {
        match self.state {
            State::Sync => {
                if byte == PACKET_HEADER {
                    self.buf[0] = byte;
                    self.pos = 1;
                    self.state = State::Header;
                }
            }
            State::Header => {
                self.buf[self.pos] = byte;
                self.pos += 1;

                if self.pos == HEADER_SIZE {
                    let len = self.buf[OFF_LEN] as usize;
                    if len > MAX_PAYLOAD {
                        self.reset();
                    } else {
                        self.expected_total = HEADER_SIZE + len + 1; // +1 for CRC
                        self.state = State::PayloadCrc;
                    }
                }
            }
            State::PayloadCrc => {
                self.buf[self.pos] = byte;
                self.pos += 1;

                if self.pos == self.expected_total {
                    let ok = parse_packet(&self.buf[..self.pos]).is_some();
                    if ok {
                        out[..self.pos].copy_from_slice(&self.buf[..self.pos]);
                        let out_len = self.pos;
                        self.reset();
                        return Some(out_len);
                    }
                    self.reset();
                }
            }
        }

        None
    }
}

impl Default for SerialFramer {
    fn default() -> Self {
        Self::new()
    }
}
