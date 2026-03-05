#pragma once

// WiFi credentials
#define WIFI_SSID "ATTGpyJKmS"
#define WIFI_PASS "cw7nqpsr?sun"

// Home location
#define HOME_LAT 30.6905
#define HOME_LON -88.1632

// ADS-B settings — reduced for CYD (no PSRAM, 320KB DRAM)
#define ADSB_RADIUS_NM 75
#define ADSB_POLL_INTERVAL_MS 5000
#define MAX_AIRCRAFT 50
#define TRAIL_LENGTH 15

// CYD display
#define LCD_H_RES 320
#define LCD_V_RES 240

// CYD touch calibration (landscape rotation 1)
#define TOUCH_X_MIN 3538
#define TOUCH_X_MAX 620
#define TOUCH_Y_MIN 3509
#define TOUCH_Y_MAX 520
