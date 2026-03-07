/// Minimal Arduino.h mock for native (desktop) testing.
/// Only the symbols actually used by testable headers are provided.
#pragma once

#include <cstdint>

#ifndef constrain
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif
