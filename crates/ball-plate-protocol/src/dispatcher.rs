//! Command dispatcher (register handlers, deduplicate by SEQ, dispatch).

#![allow(dead_code)]

use core::ffi::c_void;

use crate::command::CmdId;
use crate::packet::{parse_packet, MAX_PACKET_SIZE};

/// Callback signature.
///
/// Mirrors the C++ signature: `void (*)(void* ctx, const uint8_t* payload, uint8_t len)`.
pub type CmdHandler = fn(ctx: *mut c_void, payload: *const u8, len: usize);

/// Source of raw packets.
///
/// Implementations should be non-blocking. If no packet is available, return `None`.
pub trait CommandReceiver {
    fn receive(&mut self, buf: &mut [u8; MAX_PACKET_SIZE]) -> Option<usize>;
}

#[derive(Clone, Copy)]
struct Entry {
    cmd: CmdId,
    handler: CmdHandler,
    ctx: *mut c_void,
}

/// Receives raw packets via a `CommandReceiver`, validates them, deduplicates
/// by SEQ, and dispatches to registered handlers.
pub struct CommandDispatcher {
    handlers: [Option<Entry>; Self::MAX_HANDLERS],
    count: usize,
    last_seq: u8,
    seq_initialized: bool,
}

impl CommandDispatcher {
    pub const MAX_HANDLERS: usize = 8;
    pub const MAX_DRAIN_PER_POLL: usize = 3; // matches nRF24L01 RX FIFO depth

    pub const fn new() -> Self {
        Self {
            handlers: [None; Self::MAX_HANDLERS],
            count: 0,
            last_seq: 0,
            seq_initialized: false,
        }
    }

    /// Register a handler for a command. Re-registering the same `CmdId`
    /// overwrites the previous handler.
    pub fn on(&mut self, cmd: CmdId, handler: CmdHandler, ctx: *mut c_void) {
        // Overwrite existing entry
        for i in 0..self.count {
            if let Some(e) = self.handlers[i] {
                if e.cmd == cmd {
                    self.handlers[i] = Some(Entry { cmd, handler, ctx });
                    return;
                }
            }
        }

        if self.count < Self::MAX_HANDLERS {
            self.handlers[self.count] = Some(Entry { cmd, handler, ctx });
            self.count += 1;
        }
    }

    /// Non-blocking poll: drains up to `MAX_DRAIN_PER_POLL` packets from the
    /// receiver, validates each, deduplicates by SEQ, and dispatches.
    pub fn poll<R: CommandReceiver>(&mut self, receiver: &mut R) {
        let mut buf = [0u8; MAX_PACKET_SIZE];

        for _ in 0..Self::MAX_DRAIN_PER_POLL {
            let len = match receiver.receive(&mut buf) {
                Some(len) => len,
                None => return,
            };

            let pkt = match parse_packet(&buf[..len]) {
                Some(pkt) => pkt,
                None => continue,
            };

            // SEQ dedup: only guards against adjacent duplicates.
            if !self.seq_initialized {
                self.seq_initialized = true;
            } else if pkt.seq == self.last_seq {
                continue;
            }
            self.last_seq = pkt.seq;

            // Dispatch — unknown CmdId silently dropped
            for i in 0..self.count {
                if let Some(entry) = self.handlers[i] {
                    if entry.cmd == pkt.cmd {
                        let (payload_ptr, payload_len) = if pkt.payload.is_empty() {
                            (core::ptr::null(), 0)
                        } else {
                            (pkt.payload.as_ptr(), pkt.payload.len())
                        };
                        (entry.handler)(entry.ctx, payload_ptr, payload_len);
                        break;
                    }
                }
            }
        }
    }
}

impl Default for CommandDispatcher {
    fn default() -> Self {
        Self::new()
    }
}
