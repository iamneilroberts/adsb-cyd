#pragma once
#include <cstdint>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "../config.h"

struct TrailPoint {
    float lat;
    float lon;
    int32_t alt;
    uint32_t timestamp;
};

struct Aircraft {
    char icao_hex[7];       // e.g. "A0B1C2"
    char callsign[9];       // e.g. "UAL1234"
    char registration[9];   // e.g. "N12345"
    char type_code[5];      // e.g. "B738"
    char category[3];       // ADS-B emitter category e.g. "A3"
    char desc[40];          // type description e.g. "Boeing 737-800"
    char owner_op[32];      // operator e.g. "United Airlines"
    char origin[5];         // IATA airport code e.g. "ATL"
    char dest[5];           // IATA airport code e.g. "MDW"
    float lat;
    float lon;
    int32_t altitude;       // feet
    int16_t speed;          // knots
    int16_t heading;        // degrees 0-359
    int16_t vert_rate;      // ft/min
    uint16_t squawk;
    bool on_ground;
    bool is_military;
    bool is_emergency;
    bool is_watched;
    float mach;             // Mach number (0 = not available)
    int16_t ias;            // indicated airspeed kts (0 = n/a)
    int16_t tas;            // true airspeed kts (0 = n/a)
    int32_t nav_altitude;   // autopilot target altitude ft (0 = n/a)
    float roll;             // bank angle degrees (0 = wings level or n/a)
    float nav_qnh;          // altimeter setting hPa (0 = n/a)
    uint32_t last_seen;     // millis() timestamp
    uint32_t stale_since;   // 0 = fresh, else millis() when first went stale
    TrailPoint trail[TRAIL_LENGTH];
    uint8_t trail_count;

    void clear() {
        memset(this, 0, sizeof(Aircraft));
    }
};

#define GHOST_TIMEOUT_MS 30000

// Compute opacity for stale (ghost) aircraft: 255→0 over 30s
// Caller must pass current millis() value
static inline uint8_t compute_aircraft_opacity(uint32_t stale_since, uint32_t now_ms) {
    if (stale_since == 0) return 255;
    uint32_t elapsed = now_ms - stale_since;
    if (elapsed >= GHOST_TIMEOUT_MS) return 0;
    return (uint8_t)(255 - (elapsed * 255 / GHOST_TIMEOUT_MS));
}

// Thread-safe aircraft list
class AircraftList {
public:
    Aircraft *aircraft = nullptr;
    int count = 0;
    SemaphoreHandle_t mutex;

    void init() {
        mutex = xSemaphoreCreateMutex();
        count = 0;
        // CYD: no PSRAM, allocate in DRAM (MALLOC_CAP_8BIT = byte-addressable DRAM)
        aircraft = (Aircraft *)heap_caps_malloc(MAX_AIRCRAFT * sizeof(Aircraft), MALLOC_CAP_8BIT);
        if (aircraft) memset(aircraft, 0, MAX_AIRCRAFT * sizeof(Aircraft));
    }

    bool lock(TickType_t timeout = pdMS_TO_TICKS(100)) {
        return xSemaphoreTake(mutex, timeout) == pdTRUE;
    }

    void unlock() {
        xSemaphoreGive(mutex);
    }
};
