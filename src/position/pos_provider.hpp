#pragma once

#include <Arduino.h>

struct Position {
    float x;
    float y;
};

class PosProvider {
public:
    virtual ~PosProvider() = default;

    virtual void begin() = 0;

    /// try to read new position, return true if success and write to out
    virtual bool read(Position& out) = 0;
};
