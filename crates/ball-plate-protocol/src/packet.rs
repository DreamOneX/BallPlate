//! Packet constants and parsing helpers.

#![allow(dead_code)]

use crate::command::CmdId;
use crate::crc8::crc8;

// ── Protocol constants ─────────────────────────────────────

pub const PACKET_HEADER: u8 = 0xA5;
pub const PROTOCOL_VER: u8 = 0x01;
pub const MAX_PAYLOAD: usize = 26;
pub const MAX_PACKET_SIZE: usize = 32; // nRF24L01 payload limit

// Header field offsets
pub const OFF_HEADER: usize = 0;
pub const OFF_VER: usize = 1;
pub const OFF_CMD: usize = 2;
pub const OFF_SEQ: usize = 3;
pub const OFF_LEN: usize = 4;
pub const OFF_PAYLOAD: usize = 5;
pub const HEADER_SIZE: usize = 5; // HEADER + VER + CMD + SEQ + LEN

/// Parsed packet (views into the raw buffer).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParsedPacket<'a> {
    pub cmd: CmdId,
    pub seq: u8,
    pub payload: &'a [u8],
}

/// Validate and parse a raw byte buffer into a `ParsedPacket`.
///
/// Checks: header marker, protocol version, length bounds, CRC-8.
/// Returns `None` on any validation failure.
pub fn parse_packet(raw: &[u8]) -> Option<ParsedPacket<'_>> {
    // Minimum packet: HEADER(1) + VER(1) + CMD(1) + SEQ(1) + LEN(1) + CRC(1) = 6
    if raw.len() < HEADER_SIZE + 1 {
        return None;
    }

    if raw[OFF_HEADER] != PACKET_HEADER {
        return None;
    }
    if raw[OFF_VER] != PROTOCOL_VER {
        return None;
    }

    let len = raw[OFF_LEN] as usize;
    if len > MAX_PAYLOAD {
        return None;
    }
    if raw.len() < HEADER_SIZE + len + 1 {
        return None;
    }

    // CRC covers bytes 0 .. HEADER_SIZE+len-1
    let expected = crc8(&raw[..HEADER_SIZE + len]);
    if raw[HEADER_SIZE + len] != expected {
        return None;
    }

    let payload = if len > 0 {
        &raw[OFF_PAYLOAD..OFF_PAYLOAD + len]
    } else {
        &raw[0..0]
    };

    Some(ParsedPacket {
        cmd: CmdId(raw[OFF_CMD]),
        seq: raw[OFF_SEQ],
        payload,
    })
}

/// Build a packet into the provided fixed buffer.
/// Returns the total length written, or `None` if the payload is too large.
pub fn build_packet(
    buf: &mut [u8; MAX_PACKET_SIZE],
    cmd: CmdId,
    seq: u8,
    payload: &[u8],
) -> Option<usize> {
    if payload.len() > MAX_PAYLOAD {
        return None;
    }

    buf[OFF_HEADER] = PACKET_HEADER;
    buf[OFF_VER] = PROTOCOL_VER;
    buf[OFF_CMD] = cmd.0;
    buf[OFF_SEQ] = seq;
    buf[OFF_LEN] = payload.len() as u8;

    if !payload.is_empty() {
        buf[OFF_PAYLOAD..OFF_PAYLOAD + payload.len()].copy_from_slice(payload);
    }

    let crc = crc8(&buf[..HEADER_SIZE + payload.len()]);
    buf[HEADER_SIZE + payload.len()] = crc;

    Some(HEADER_SIZE + payload.len() + 1)
}

/// Read a little-endian IEEE754 `f32` from the first 4 bytes of `data`.
///
/// This is alignment-safe.
pub fn read_f32_le(data: &[u8]) -> f32 {
    let bytes: [u8; 4] = data[..4].try_into().unwrap();
    f32::from_le_bytes(bytes)
}
