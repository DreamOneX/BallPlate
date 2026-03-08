#pragma once

namespace ball_plate {

class IServo {
public:
    virtual ~IServo() = default;

    IServo(const IServo&) = delete;
    IServo& operator=(const IServo&) = delete;

    virtual void write(float angle) = 0;

    virtual void setOffset(float offset) = 0;

protected:
    IServo() = default;
};

} // namespace ball_plate