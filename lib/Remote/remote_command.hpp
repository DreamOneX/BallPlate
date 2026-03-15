#pragma once

#include <stdint.h>
#include <string.h>

namespace ball_plate {

// ── Command & mode enums ───────────────────────────────────
enum class CmdId : uint8_t {
    REMOTE_STOP   = 0x01,
    REMOTE_RESUME = 0x02,
    TARGET_UPDATE = 0x03,
};

enum class StopMode : uint8_t {
    FREEZE = 0x01,  // servo 保持当前角度
    PARK   = 0x02,  // servo 回 SERVO_CENTER
};

// ── Protocol constants ─────────────────────────────────────
constexpr uint8_t PACKET_HEADER   = 0xA5;
constexpr uint8_t PROTOCOL_VER    = 0x01;
constexpr uint8_t MAX_PAYLOAD     = 26;
constexpr uint8_t MAX_PACKET_SIZE = 32;  // nRF24L01 payload limit

// Header field offsets (no packed struct — avoids alignment / strict-aliasing issues)
constexpr uint8_t OFF_HEADER  = 0;
constexpr uint8_t OFF_VER     = 1;
constexpr uint8_t OFF_CMD     = 2;
constexpr uint8_t OFF_SEQ     = 3;
constexpr uint8_t OFF_LEN     = 4;
constexpr uint8_t OFF_PAYLOAD = 5;
constexpr uint8_t HEADER_SIZE = 5;  // HEADER + VER + CMD + SEQ + LEN

// ── Parsed packet (value type, filled by parsePacket) ──────
struct ParsedPacket {
    CmdId          cmd;
    uint8_t        seq;
    uint8_t        payloadLen;
    const uint8_t* payload;  // points into the raw buffer
};

// ── CRC-8 (poly 0x07, init 0x00) ──────────────────────────
inline uint8_t crc8(const uint8_t* data, uint8_t length) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
        }
    }
    return crc;
}

/// Validate and parse a raw byte buffer into a ParsedPacket.
///
/// Checks: header marker, protocol version, length bounds, CRC-8.
/// Returns false on any validation failure (caller should discard).
inline bool parsePacket(const uint8_t* raw, uint8_t rawLen, ParsedPacket& out) {
    // Minimum packet: HEADER(1) + VER(1) + CMD(1) + SEQ(1) + LEN(1) + CRC(1) = 6
    if (rawLen < HEADER_SIZE + 1) return false;

    if (raw[OFF_HEADER] != PACKET_HEADER) return false;
    if (raw[OFF_VER]    != PROTOCOL_VER)  return false;

    uint8_t len = raw[OFF_LEN];
    if (len > MAX_PAYLOAD)              return false;
    if (rawLen < HEADER_SIZE + len + 1) return false;  // +1 for CRC

    // CRC covers bytes 0 .. HEADER_SIZE+len-1
    uint8_t expected = crc8(raw, HEADER_SIZE + len);
    if (raw[HEADER_SIZE + len] != expected) return false;

    out.cmd        = static_cast<CmdId>(raw[OFF_CMD]);
    out.seq        = raw[OFF_SEQ];
    out.payloadLen = len;
    out.payload    = (len > 0) ? &raw[OFF_PAYLOAD] : nullptr;
    return true;
}

/// Safe float deserialization via memcpy (no alignment / strict-aliasing issues).
/// Compiler optimizes this to a single LDR on Cortex-M4.
inline float readFloat(const uint8_t* p) {
    float v;
    memcpy(&v, p, sizeof(float));
    return v;
}

} // namespace ball_plate
