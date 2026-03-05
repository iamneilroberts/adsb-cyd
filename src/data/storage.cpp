#include "storage.h"
#include "../config.h"
#include <Preferences.h>
#include <cstring>

UserConfig g_config = {};
static Preferences _prefs;

UserConfig storage_load_config() {
    UserConfig cfg;

    // Compiled defaults
    strncpy(cfg.wifi_ssid, WIFI_SSID, sizeof(cfg.wifi_ssid) - 1);
    cfg.wifi_ssid[sizeof(cfg.wifi_ssid) - 1] = '\0';
    strncpy(cfg.wifi_pass, WIFI_PASS, sizeof(cfg.wifi_pass) - 1);
    cfg.wifi_pass[sizeof(cfg.wifi_pass) - 1] = '\0';
    cfg.home_lat = HOME_LAT;
    cfg.home_lon = HOME_LON;
    cfg.radius_nm = ADSB_RADIUS_NM;
    cfg.use_metric = false;
    cfg.use_ethernet = false; // WiFi by default
    cfg.watchlist_count = 0;
    cfg.alert_military = true;
    cfg.alert_emergency = true;
    cfg.alert_autofocus = true; // auto-switch to map on mil/emg alerts
    cfg.cycle_enabled = true;
    cfg.cycle_interval_s = 30;
    cfg.cycle_inactivity_s = 60;
    cfg.trails_enabled = true;
    cfg.trail_max_points = 30;
    cfg.trail_style = 0;
    cfg.brightness = 255;
    cfg.night_mode = false;
    cfg.map_zoom_idx = 1;    // 50nm default
    cfg.radar_zoom_idx = 0;  // 100nm default
    cfg.arrivals_filter_idx = 4; // ALL default

    _prefs.begin("adsb", true); // read-only

    // Override with NVS values where they exist
    if (_prefs.isKey("ssid"))
        strlcpy(cfg.wifi_ssid, _prefs.getString("ssid", cfg.wifi_ssid).c_str(), sizeof(cfg.wifi_ssid));
    if (_prefs.isKey("pass"))
        strlcpy(cfg.wifi_pass, _prefs.getString("pass", cfg.wifi_pass).c_str(), sizeof(cfg.wifi_pass));
    cfg.home_lat = _prefs.getFloat("lat", cfg.home_lat);
    cfg.home_lon = _prefs.getFloat("lon", cfg.home_lon);
    cfg.radius_nm = _prefs.getInt("radius", cfg.radius_nm);
    cfg.use_metric = _prefs.getBool("metric", cfg.use_metric);
    cfg.use_ethernet = _prefs.getBool("use_eth", cfg.use_ethernet);
    cfg.alert_military = _prefs.getBool("alrt_mil", cfg.alert_military);
    cfg.alert_emergency = _prefs.getBool("alrt_emg", cfg.alert_emergency);
    cfg.alert_autofocus = _prefs.getBool("alrt_af", cfg.alert_autofocus);
    cfg.cycle_enabled = _prefs.getBool("cyc_on", cfg.cycle_enabled);
    cfg.cycle_interval_s = _prefs.getInt("cyc_int", cfg.cycle_interval_s);
    cfg.cycle_inactivity_s = _prefs.getInt("cyc_idle", cfg.cycle_inactivity_s);
    cfg.trails_enabled = _prefs.getBool("trail_on", cfg.trails_enabled);
    cfg.trail_max_points = _prefs.getInt("trail_pts", cfg.trail_max_points);
    cfg.trail_style = _prefs.getInt("trail_sty", cfg.trail_style);
    cfg.brightness = _prefs.getUChar("bright", cfg.brightness);
    cfg.night_mode = _prefs.getBool("night", cfg.night_mode);
    cfg.map_zoom_idx = _prefs.getInt("map_zoom", cfg.map_zoom_idx);
    cfg.radar_zoom_idx = _prefs.getInt("rdr_zoom", cfg.radar_zoom_idx);
    cfg.arrivals_filter_idx = _prefs.getInt("arr_filt", cfg.arrivals_filter_idx);

    _prefs.end();
    Serial.println("Storage: config loaded from NVS");
    return cfg;
}

void storage_save_config(const UserConfig &cfg) {
    _prefs.begin("adsb", false); // read-write

    _prefs.putString("ssid", cfg.wifi_ssid);
    _prefs.putString("pass", cfg.wifi_pass);
    _prefs.putFloat("lat", cfg.home_lat);
    _prefs.putFloat("lon", cfg.home_lon);
    _prefs.putInt("radius", cfg.radius_nm);
    _prefs.putBool("metric", cfg.use_metric);
    _prefs.putBool("use_eth", cfg.use_ethernet);
    _prefs.putBool("alrt_mil", cfg.alert_military);
    _prefs.putBool("alrt_emg", cfg.alert_emergency);
    _prefs.putBool("alrt_af", cfg.alert_autofocus);
    _prefs.putBool("cyc_on", cfg.cycle_enabled);
    _prefs.putInt("cyc_int", cfg.cycle_interval_s);
    _prefs.putInt("cyc_idle", cfg.cycle_inactivity_s);
    _prefs.putBool("trail_on", cfg.trails_enabled);
    _prefs.putInt("trail_pts", cfg.trail_max_points);
    _prefs.putInt("trail_sty", cfg.trail_style);
    _prefs.putUChar("bright", cfg.brightness);
    _prefs.putBool("night", cfg.night_mode);
    _prefs.putInt("map_zoom", cfg.map_zoom_idx);
    _prefs.putInt("rdr_zoom", cfg.radar_zoom_idx);
    _prefs.putInt("arr_filt", cfg.arrivals_filter_idx);

    _prefs.end();
    Serial.println("Storage: config saved to NVS");
}
