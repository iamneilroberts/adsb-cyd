#pragma once
#include <Arduino.h>

// User-Agent of the form "adsb-cyd-radar/1.1 (a1b2c3)", with the suffix taken
// from the chip's MAC. Built once (thread-safe function-local static) and stable
// for the life of the device.
inline const char *user_agent() {
    static const String ua = [] {
        char buf[48];
        snprintf(buf, sizeof(buf), "adsb-cyd-radar/1.1 (%06x)",
                 (unsigned)(ESP.getEfuseMac() & 0xFFFFFF));
        return String(buf);
    }();
    return ua.c_str();
}
