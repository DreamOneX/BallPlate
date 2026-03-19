#![no_std]

pub mod command;
pub mod crc8;
pub mod dispatcher;
pub mod framer;
pub mod packet;

pub use command::{CmdId, StopMode};
pub use crc8::crc8;
pub use dispatcher::{CmdHandler, CommandDispatcher, CommandReceiver};
pub use framer::SerialFramer;
pub use packet::{
    build_packet, parse_packet, read_f32_le, ParsedPacket, HEADER_SIZE, MAX_PACKET_SIZE, MAX_PAYLOAD,
    OFF_CMD, OFF_HEADER, OFF_LEN, OFF_PAYLOAD, OFF_SEQ, OFF_VER, PACKET_HEADER, PROTOCOL_VER,
};

#[cfg(test)]
mod tests;
