#pragma once

#include <Arduino.h>
#include "command_receiver.hpp"
#include "remote_command.hpp"

namespace ball_plate {

/// ICommandReceiver implementation over a byte-stream serial port.
///
/// Unlike nRF24L01 (packet-oriented), serial is a byte stream.
/// This class uses a state machine to frame incoming bytes into
/// complete packets:  SYNC → HEADER → PAYLOAD+CRC → validate.
class SerialReceiver : public ICommandReceiver {
public:
    explicit SerialReceiver(HardwareSerial& serial, uint32_t baud = 115200)
        : _serial(serial), _baud(baud) {}

    void begin() override {
        _serial.begin(_baud);
    }

    bool receive(uint8_t* buf, uint8_t maxLen, uint8_t& outLen) override {
        // Process up to a bounded number of bytes per call to stay non-blocking.
        uint8_t budget = MAX_BYTES_PER_CALL;

        while (_serial.available() > 0 && budget-- > 0) {
            uint8_t byte = _serial.read();

            switch (_state) {
            case State::SYNC:
                if (byte == PACKET_HEADER) {
                    _buf[0] = byte;
                    _pos    = 1;
                    _state  = State::HEADER;
                }
                // else: discard non-header bytes
                break;

            case State::HEADER:
                _buf[_pos++] = byte;
                if (_pos == HEADER_SIZE) {
                    uint8_t len = _buf[OFF_LEN];
                    if (len > MAX_PAYLOAD) {
                        reset();  // invalid length → resync
                    } else {
                        _expectedTotal = HEADER_SIZE + len + 1;  // +1 for CRC
                        _state = State::PAYLOAD_CRC;
                    }
                }
                break;

            case State::PAYLOAD_CRC:
                _buf[_pos++] = byte;
                if (_pos == _expectedTotal) {
                    // Complete frame received — validate and output
                    ParsedPacket pkt;
                    if (parsePacket(_buf, _pos, pkt) &&
                        _pos <= maxLen) {
                        memcpy(buf, _buf, _pos);
                        outLen = _pos;
                        reset();
                        return true;
                    }
                    // CRC or validation failed → resync
                    reset();
                }
                break;
            }
        }
        return false;
    }

private:
    static constexpr uint8_t MAX_BYTES_PER_CALL = MAX_PACKET_SIZE * 2;

    enum class State : uint8_t {
        SYNC,         // scanning for 0xA5
        HEADER,       // accumulating VER + CMD + SEQ + LEN
        PAYLOAD_CRC,  // accumulating payload bytes + CRC
    };

    void reset() {
        _state = State::SYNC;
        _pos   = 0;
    }

    HardwareSerial& _serial;
    uint32_t        _baud;
    uint8_t         _buf[MAX_PACKET_SIZE] = {};
    uint8_t         _pos           = 0;
    uint8_t         _expectedTotal = 0;
    State           _state         = State::SYNC;
};

} // namespace ball_plate
