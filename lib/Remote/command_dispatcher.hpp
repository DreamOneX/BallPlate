#pragma once

#include "command_receiver.hpp"
#include "remote_command.hpp"

namespace ball_plate {

/// Callback signature: ctx is the opaque pointer registered with on().
using CmdHandler = void (*)(void* ctx, const uint8_t* payload, uint8_t len);

/// Receives raw packets via ICommandReceiver, validates them, deduplicates
/// by SEQ, and dispatches to registered command handlers.
class CommandDispatcher {
public:
    /// Register a handler for a command.  Re-registering the same CmdId
    /// overwrites the previous handler.
    void on(CmdId cmd, CmdHandler handler, void* ctx = nullptr) {
        // Overwrite existing entry for this cmd
        for (uint8_t i = 0; i < _count; ++i) {
            if (_handlers[i].cmd == cmd) {
                _handlers[i].handler = handler;
                _handlers[i].ctx     = ctx;
                return;
            }
        }
        if (_count < MAX_HANDLERS) {
            _handlers[_count++] = { cmd, handler, ctx };
        }
    }

    /// Non-blocking poll: drains up to MAX_DRAIN_PER_POLL packets from the
    /// receiver, validates each, deduplicates by SEQ, and dispatches.
    void poll(ICommandReceiver& receiver) {
        uint8_t buf[MAX_PACKET_SIZE];

        for (uint8_t drain = 0; drain < MAX_DRAIN_PER_POLL; ++drain) {
            uint8_t len = 0;
            if (!receiver.receive(buf, sizeof(buf), len)) return;

            ParsedPacket pkt;
            if (!parsePacket(buf, len, pkt)) continue;

            // SEQ dedup: only guards against adjacent duplicates
            // (e.g. nRF24L01 auto-retransmit).  All current commands are
            // idempotent, so occasional duplicate execution is harmless.
            if (!_seqInitialized) {
                _seqInitialized = true;
            } else if (pkt.seq == _lastSeq) {
                continue;
            }
            _lastSeq = pkt.seq;

            // Dispatch — unknown CmdId silently dropped
            for (uint8_t i = 0; i < _count; ++i) {
                if (_handlers[i].cmd == pkt.cmd) {
                    _handlers[i].handler(_handlers[i].ctx,
                                         pkt.payload, pkt.payloadLen);
                    break;
                }
            }
        }
    }

private:
    static constexpr uint8_t MAX_HANDLERS       = 8;
    static constexpr uint8_t MAX_DRAIN_PER_POLL  = 3;  // matches nRF24L01 RX FIFO depth

    struct Entry {
        CmdId      cmd;
        CmdHandler handler;
        void*      ctx;
    };

    Entry   _handlers[MAX_HANDLERS] = {};
    uint8_t _count          = 0;
    uint8_t _lastSeq        = 0;
    bool    _seqInitialized = false;
};

} // namespace ball_plate
