//! CRC-8 implementation (poly 0x07, init 0x00).

#![allow(dead_code)]

/// CRC-8 with polynomial 0x07 and initial value 0x00.
///
/// Direct port of `remote_command.hpp`.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}
