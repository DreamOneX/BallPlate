#pragma once

#include "pos_provider.hpp"

// Text protocol: `X,Y\n`, comma-separated floats, newline terminated
class UartPosProvider : public PosProvider {
public:
    explicit UartPosProvider(HardwareSerial& serial, uint32_t baud = 115200)
        : _serial(serial), _baud(baud) {}

    void begin() override {
        _serial.begin(_baud);
    }

    bool read(Position& out) override {
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

private:
    static bool parse(const char* line, Position& out) {
        char* comma = strchr(line, ',');
        if (!comma) return false;

        out.x = atof(line);
        out.y = atof(comma + 1);
        return true;
    }

    HardwareSerial& _serial;
    uint32_t _baud;
    char _buf[32] = {};
    uint8_t _len  = 0;
};
