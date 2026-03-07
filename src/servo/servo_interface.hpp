#pragma once

class ServoInterface {
public:
    virtual ~ServoInterface() = default;

    ServoInterface(const ServoInterface&) = delete;
    ServoInterface& operator=(const ServoInterface&) = delete;

    virtual void write(float angle) = 0;

    virtual void setOffset(float offset) = 0;

protected:
    ServoInterface() = default;
};