#pragma once

#include <config.hpp>
#include <command_dispatcher.hpp>
#include "../control_loop.hpp"

namespace ball_plate {

/// Bind remote command handlers to the control loop.
///
/// All handlers are non-capturing lambdas that cast the void* ctx back to
/// ControlLoop*.  They only set volatile flags / pending values — no blocking
/// I/O, no direct PID mutation (avoiding ISR/loop data races).
inline void setupRemoteHandlers(CommandDispatcher& d, ControlLoop& control) {
    d.on(CmdId::REMOTE_STOP,
        [](void* ctx, const uint8_t* p, uint8_t len) {
            if (len < 1) return;
            auto mode = static_cast<StopMode>(p[0]);
            if (mode != StopMode::FREEZE && mode != StopMode::PARK) return;
            static_cast<ControlLoop*>(ctx)->remoteStop(mode);
        }, &control);

    d.on(CmdId::REMOTE_RESUME,
        [](void* ctx, const uint8_t*, uint8_t) {
            if constexpr (ALLOW_REMOTE_RESUME) {
                static_cast<ControlLoop*>(ctx)->requestResume();
            }
        }, &control);

    d.on(CmdId::TARGET_UPDATE,
        [](void* ctx, const uint8_t* p, uint8_t len) {
            if (len < 8) return;
            float x = readFloat(p);
            float y = readFloat(p + 4);
            static_cast<ControlLoop*>(ctx)->requestTargetUpdate(x, y);
        }, &control);
}

} // namespace ball_plate
