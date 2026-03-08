#pragma once

#include "pos_provider.hpp"

namespace ball_plate {

// Text protocol: `X,Y\n`, comma-separated floats, newline terminated
class UartPosProvider : public IPosProvider {
public:
    explicit UartPosProvider(HardwareSerial& serial, uint32_t baud = 115200)
        : _serial(serial), _baud(baud) {}

    void begin() override {
        _serial.begin(_baud);
    }

    /// Call frequently from loop() to drain UART buffer and update latest position
    void update() override {
        Position pos;
        while (read(pos)) {
            noInterrupts();
            _latest = pos;
            _hasNew = true;
            interrupts();
        }
    }

    /// ISR-safe: atomically copy the latest position (returns true if new data available)
    bool getLatest(Position& out) override {
        noInterrupts();
        bool ok = _hasNew;
        if (ok) {
            out = _latest;
            _hasNew = false;
        }
        interrupts();
        return ok;
    }

private:
    static bool parse(const char* line, Position& out) {
        char* comma = strchr(line, ',');
        if (!comma) return false;

        out.x = atof(line);
        out.y = atof(comma + 1);
        return true;
    }

    bool read(Position& out) {
        while (_serial.available()) {
            char c = _serial.read();

            if (c == '\n') {
                _buf[_len] = '\0';
                bool ok = parse(_buf, out);
                _len = 0;
                return ok;
            }

            if (_len < sizeof(_buf) - 1)
                _buf[_len++] = c;
        }
        return false;
    }


    HardwareSerial& _serial;
    uint32_t _baud;
    char _buf[32] = {};
    uint8_t _len  = 0;

    Position _latest = {};   // guarded by noInterrupts()
    volatile bool _hasNew = false;
};

} // namespace ball_plate
