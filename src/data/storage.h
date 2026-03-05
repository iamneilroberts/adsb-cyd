#pragma once
#include <cstdint>

struct UserConfig {
    char wifi_ssid[33];
    char wifi_pass[65];
    float home_lat;
    float home_lon;
    int radius_nm;
    bool use_metric;
    bool use_ethernet;       // true=Ethernet, false=WiFi (default: WiFi)
    char watchlist[10][7]; // up to 10 ICAO hex codes
    int watchlist_count;

    // View cycle settings
    bool cycle_enabled;
    int cycle_interval_s;    // seconds between auto-advance (default 30)
    int cycle_inactivity_s;  // seconds before resuming cycling after touch (default 60)

    // Alert settings
    bool alert_military;     // show popup for military aircraft
    bool alert_emergency;    // show popup for squawk 7500/7600/7700
    bool alert_autofocus;    // auto-switch to map on military/emergency alerts

    // Trail settings
    bool trails_enabled;
    int trail_max_points;    // 10-60 (default 30)
    int trail_style;         // 0=line, 1=dots

    // Display settings
    uint8_t brightness;      // PWM brightness 32-255 (default 255)
    bool night_mode;         // amber/red palette (default false)

    // Zoom indices (persisted per-view)
    int map_zoom_idx;
    int radar_zoom_idx;
    int arrivals_filter_idx; // distance filter index for arrivals
};

// Load config from NVS. Returns defaults if not found.
UserConfig storage_load_config();

// Save config to NVS
void storage_save_config(const UserConfig &cfg);

// Global runtime config — loaded at boot, updated on settings save
extern UserConfig g_config;
