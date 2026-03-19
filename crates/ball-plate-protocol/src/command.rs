//! Command identifiers and associated enums.

#![allow(dead_code)]

/// Command identifier (u8) for the remote protocol.
///
/// This is a newtype rather than a Rust `enum` so that unknown command IDs can
/// be preserved (the C++ implementation accepts unknown IDs at parse stage and
/// drops them at dispatch stage).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct CmdId(pub u8);

impl CmdId {
    pub const REMOTE_STOP: Self = Self(0x01);
    pub const REMOTE_RESUME: Self = Self(0x02);
    pub const TARGET_UPDATE: Self = Self(0x03);

    // Reserved / documented in PROTOCOL.md
    pub const HEARTBEAT: Self = Self(0x10);
    pub const PID_TUNE: Self = Self(0x11);
}

impl From<CmdId> for u8 {
    fn from(value: CmdId) -> Self {
        value.0
    }
}

/// Stop modes for `REMOTE_STOP`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StopMode {
    Freeze = 0x01,
    Park = 0x02,
}

impl TryFrom<u8> for StopMode {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(Self::Freeze),
            0x02 => Ok(Self::Park),
            _ => Err(()),
        }
    }
}
