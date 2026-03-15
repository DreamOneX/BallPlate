#pragma once

#include <stdint.h>

namespace ball_plate {

/// Abstract interface for receiving complete command packets.
///
/// Implementations wrap a specific transport (nRF24L01, Serial, BLE, …).
/// The contract is packet-oriented: each successful receive() returns
/// exactly one complete raw packet (header through CRC).
class ICommandReceiver {
public:
    virtual ~ICommandReceiver() = default;

    virtual void begin() = 0;

    /// Non-blocking attempt to receive one complete packet.
    ///
    /// @param buf     Destination buffer (must be >= MAX_PACKET_SIZE bytes)
    /// @param maxLen  Size of buf
    /// @param outLen  Set to actual packet length on success
    /// @return true   if a packet was retrieved; false if none available.
    ///
    /// If multiple packets are queued, each call returns the oldest one.
    /// Note: the underlying buffer depth is finite (nRF24L01 FIFO holds
    /// only 3 packets); overflow causes hardware-level packet loss.
    virtual bool receive(uint8_t* buf, uint8_t maxLen, uint8_t& outLen) = 0;
};

} // namespace ball_plate
