#pragma once

#include <Arduino.h>

namespace ball_plate {

struct Position {
    float x;
    float y;
};

class IPosProvider {
public:
    virtual ~IPosProvider() = default;

    virtual void begin() = 0;

    virtual void update() = 0;

    virtual bool getLatest(Position& out) = 0;

    /// try to read new position, return true if success and write to out
    virtual bool read(Position& out) = 0;
};

} // namespace ball_plate
