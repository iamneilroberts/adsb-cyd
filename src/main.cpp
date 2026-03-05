// ADS-B Radar Display — CYD (ESP32-2432S028)
// 320x240 landscape, TFT_eSPI direct rendering, no LVGL
// Features: radar, arrivals, stats, detail, flight log
//   compass rose, auto-cycle, brightness, night mode,
//   sort modes, filters, mil/emg alerts, closest approach record
#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <WiFi.h>

#include "config.h"
#include "data/aircraft.h"
#include "data/fetcher.h"
#include "data/storage.h"
#include "data/error_log.h"
#include "data/http_mutex.h"
#include "data/enrichment.h"
#include "ui/filters.h"

// Touch on separate VSPI bus
#define XPT2046_IRQ   36
#define XPT2046_MOSI  32
#define XPT2046_MISO  39
#define XPT2046_CLK   25
#define XPT2046_CS    33

TFT_eSPI tft = TFT_eSPI();
SPIClass touchSPI = SPIClass(VSPI);

static AircraftList aircraft_list;

// ---- Color palette ----

static constexpr uint16_t c565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

struct ColorPalette {
    uint16_t bg;
    uint16_t grid;
    uint16_t ring;
    uint16_t sweep;
    uint16_t text;
    uint16_t trail;
    uint16_t blip;
    uint16_t blip_mil;
    uint16_t blip_emg;
    uint16_t status;
    uint16_t hdr;
    uint16_t status_bg;
    uint16_t fade_bright;
    uint16_t fade_med;
    uint16_t fade_dim;
    uint16_t fade_mil_dim;
    uint16_t fade_emg_dim;
};

static const ColorPalette PALETTE_GREEN = {
    TFT_BLACK,           // bg
    c565(0, 40, 0),      // grid
    c565(0, 60, 0),      // ring
    c565(0, 120, 0),     // sweep
    c565(0, 200, 0),     // text
    c565(0, 80, 0),      // trail
    TFT_GREEN,           // blip
    c565(255, 60, 60),   // blip_mil
    c565(255, 200, 0),   // blip_emg
    c565(0, 180, 0),     // status
    c565(0, 150, 0),     // hdr
    c565(0, 20, 0),      // status_bg
    c565(0, 255, 0),     // fade_bright
    c565(0, 140, 0),     // fade_med
    c565(0, 70, 0),      // fade_dim
    c565(120, 30, 30),   // fade_mil_dim
    c565(120, 90, 0),    // fade_emg_dim
};

static const ColorPalette PALETTE_NIGHT = {
    TFT_BLACK,           // bg
    c565(40, 10, 0),     // grid
    c565(60, 15, 0),     // ring
    c565(120, 40, 0),    // sweep
    c565(200, 80, 0),    // text
    c565(80, 30, 0),     // trail
    c565(255, 140, 0),   // blip (amber)
    c565(255, 60, 60),   // blip_mil
    c565(255, 50, 50),   // blip_emg (red)
    c565(180, 70, 0),    // status
    c565(150, 60, 0),    // hdr
    c565(20, 5, 0),      // status_bg
    c565(255, 140, 0),   // fade_bright
    c565(140, 60, 0),    // fade_med
    c565(70, 25, 0),     // fade_dim
    c565(120, 30, 30),   // fade_mil_dim
    c565(120, 20, 20),   // fade_emg_dim
};

static const ColorPalette *pal = &PALETTE_GREEN;

// ---- Geometry ----

#define STATUS_H  20
#define RADAR_Y   STATUS_H
#define RADAR_H   (LCD_V_RES - STATUS_H)
#define RADAR_CX  (LCD_H_RES / 2)
#define RADAR_CY  (RADAR_Y + RADAR_H / 2)
#define RADAR_R   (RADAR_H / 2 - 4)

// Range options (nm)
static const float RANGES[] = {150, 100, 50, 20, 5};
static const int NUM_RANGES = 5;
static int range_idx = 1;

// ---- View state ----

enum ViewMode { VIEW_LOADING, VIEW_RADAR, VIEW_ARRIVALS, VIEW_STATS, VIEW_DETAIL, VIEW_LOG, VIEW_SETTINGS };
static ViewMode current_view = VIEW_LOADING;
static bool view_changed = true;
static int detail_idx = 0;

// Sweep
static float sweep_angle = 0;
static float prev_sweep_angle = -1;
#define SWEEP_SPEED 3.0f

// ---- Differential rendering ----

struct BlipState {
    int16_t x, y;
    int16_t hx, hy;
    uint8_t r;
    bool has_heading;
    bool has_label;
};
static BlipState prev_blips[MAX_AIRCRAFT];
static int prev_blip_count = 0;
static bool radar_needs_full_redraw = true;

struct TrailState {
    int16_t x[TRAIL_LENGTH];
    int16_t y[TRAIL_LENGTH];
    uint8_t count;
};
static TrailState prev_trails[MAX_AIRCRAFT];
static int prev_trail_count = 0;

// ---- Auto-cycle ----

static uint32_t last_touch_time = 0;
static uint32_t last_cycle_time = 0;

// ---- Arrivals sort ----

enum ArrSort { SORT_DIST, SORT_ALT, SORT_SPEED };
static ArrSort arr_sort = SORT_DIST;
static const char *SORT_LABELS[] = {"DST", "ALT", "SPD"};

// ---- Alert state ----

struct AlertState {
    bool active;
    uint32_t start_time;
    char message[24];
    uint16_t color;
};
static AlertState alert = {};
#define ALERT_DURATION_MS 5000
#define ALERT_DEDUP_SIZE 8
static uint32_t alerted_icaos[ALERT_DEDUP_SIZE];
static int alerted_idx = 0;

// ---- Closest approach record ----

struct ClosestRecord {
    char callsign[9];
    char type_code[5];
    float distance_nm;
    int32_t altitude;
    uint32_t timestamp;
};
static ClosestRecord closest_record = {"---", "---", 9999.0f, 0, 0};

// ---- Flight log ----

struct LogEntry {
    uint32_t icao_hash;
    char callsign[9];
    char type_code[5];
    uint32_t first_seen;
    float closest_nm;
    bool is_military;
    bool is_emergency;
};
#define LOG_SIZE 50
static LogEntry flight_log[LOG_SIZE];
static int log_count = 0;
static int log_write_idx = 0;
static int log_page = 0;

// ---- Long-press detection ----

static uint32_t touch_down_time = 0;
static bool long_press_fired = false;
static int saved_tx = 0;

// ---- Settings view ----
#define NUM_SETTINGS 7
static int settings_sel = 0;  // selected row
static const char *SETTINGS_LABELS[] = {
    "AUTO CYCLE", "CYCLE INTERVAL", "INACTIVITY PAUSE",
    "ALERT MILITARY", "ALERT EMERGENCY", "TRAILS", "TRAIL STYLE"
};
static const int CYCLE_INTERVALS[] = {15, 30, 60, 90};
static const int INACTIVITY_VALS[] = {30, 60, 120};
#define LONG_PRESS_MS 1000

// ---- Helpers ----

static float distance_nm(float lat, float lon) {
    float dlat = lat - HOME_LAT;
    float dlon = lon - HOME_LON;
    float cos_lat = cosf(HOME_LAT * M_PI / 180.0f);
    float nm_north = dlat * 60.0f;
    float nm_east = dlon * 60.0f * cos_lat;
    return sqrtf(nm_north * nm_north + nm_east * nm_east);
}

static bool latlon_to_radar(float lat, float lon, int &px, int &py) {
    float range_nm = RANGES[range_idx];
    float dlat = lat - HOME_LAT;
    float dlon = lon - HOME_LON;
    float cos_lat = cosf(HOME_LAT * M_PI / 180.0f);
    float nm_north = dlat * 60.0f;
    float nm_east = dlon * 60.0f * cos_lat;
    if (sqrtf(nm_north * nm_north + nm_east * nm_east) > range_nm) return false;
    float scale = RADAR_R / range_nm;
    px = RADAR_CX + (int)(nm_east * scale);
    py = RADAR_CY - (int)(nm_north * scale);
    return true;
}

static uint32_t hash_icao(const char *hex) {
    uint32_t h = 0;
    for (int i = 0; hex[i]; i++) h = h * 31 + hex[i];
    return h;
}

// ---- Touch ----

static uint16_t touchRead16(uint8_t cmd) {
    digitalWrite(XPT2046_CS, LOW);
    touchSPI.beginTransaction(SPISettings(2500000, MSBFIRST, SPI_MODE0));
    touchSPI.transfer(cmd);
    uint16_t val = touchSPI.transfer16(0);
    touchSPI.endTransaction();
    digitalWrite(XPT2046_CS, HIGH);
    return val >> 3;
}

static bool getTouchPoint(int &tx, int &ty) {
    if (digitalRead(XPT2046_IRQ) != LOW) return false;
    uint32_t sumX = 0, sumY = 0;
    for (int i = 0; i < 4; i++) {
        sumX += touchRead16(0xD0);
        sumY += touchRead16(0x90);
    }
    tx = map(sumX / 4, TOUCH_X_MIN, TOUCH_X_MAX, 0, 319);
    ty = map(sumY / 4, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, 239);
    tx = constrain(tx, 0, 319);
    ty = constrain(ty, 0, 239);
    return true;
}

// ---- Brightness ----

static void apply_brightness() {
    ledcWrite(TFT_BL, g_config.brightness);
}

static void adjust_brightness(int delta) {
    int b = (int)g_config.brightness + delta;
    if (b < 32) b = 32;
    if (b > 255) b = 255;
    g_config.brightness = (uint8_t)b;
    apply_brightness();
    storage_save_config(g_config);
}

// ---- Night mode ----

static void toggle_night_mode() {
    g_config.night_mode = !g_config.night_mode;
    pal = g_config.night_mode ? &PALETTE_NIGHT : &PALETTE_GREEN;
    storage_save_config(g_config);
    // Force full redraw of everything
    tft.fillScreen(pal->bg);
    radar_needs_full_redraw = true;
    view_changed = true;
}

// ---- View cycling ----

static void cycle_view() {
    switch (current_view) {
        case VIEW_RADAR:    current_view = VIEW_ARRIVALS;  view_changed = true; break;
        case VIEW_ARRIVALS: current_view = VIEW_STATS;     view_changed = true; break;
        case VIEW_STATS:    current_view = VIEW_LOG;       view_changed = true; break;
        case VIEW_LOG:      current_view = VIEW_SETTINGS;  view_changed = true; break;
        case VIEW_SETTINGS: current_view = VIEW_DETAIL;    view_changed = true; detail_idx = 0; break;
        case VIEW_DETAIL:   current_view = VIEW_RADAR;     radar_needs_full_redraw = true; break;
        default: break;
    }
}

// ---- Touch handlers per view ----

static void handle_touch_radar(int tx) {
    if (tx < LCD_H_RES / 3) {
        // Left: cycle filter
        filter_cycle();
        radar_needs_full_redraw = true;
    } else if (tx > LCD_H_RES * 2 / 3) {
        // Right: cycle range
        int old = range_idx;
        range_idx = (range_idx + 1) % NUM_RANGES;
        if (range_idx != old) radar_needs_full_redraw = true;
    } else {
        cycle_view();
    }
}

static void handle_touch_arrivals(int tx) {
    if (tx < LCD_H_RES / 3) {
        // Left: cycle sort
        arr_sort = (ArrSort)((arr_sort + 1) % 3);
        view_changed = true;
    } else if (tx > LCD_H_RES * 2 / 3) {
        // Right: cycle range
        range_idx = (range_idx + 1) % NUM_RANGES;
        view_changed = true;
    } else {
        cycle_view();
    }
}

static void handle_touch_stats(int tx, bool is_long_press) {
    if (tx < LCD_H_RES / 3) {
        adjust_brightness(-32);
        view_changed = true;
    } else if (tx > LCD_H_RES * 2 / 3) {
        adjust_brightness(32);
        view_changed = true;
    } else {
        if (is_long_press) {
            toggle_night_mode();
        } else {
            cycle_view();
        }
    }
}

static void handle_touch_detail(int tx) {
    if (tx < LCD_H_RES / 3) {
        detail_idx--;
        view_changed = true;
    } else if (tx > LCD_H_RES * 2 / 3) {
        detail_idx++;
        view_changed = true;
    } else {
        cycle_view();
    }
}

static void handle_touch_log(int tx) {
    if (tx < LCD_H_RES / 3) {
        if (log_page > 0) log_page--;
        view_changed = true;
    } else if (tx > LCD_H_RES * 2 / 3) {
        log_page++;
        view_changed = true;
    } else {
        cycle_view();
    }
}

static void settings_adjust_selected() {
    switch (settings_sel) {
        case 0: // Auto cycle on/off
            g_config.cycle_enabled = !g_config.cycle_enabled;
            break;
        case 1: { // Cycle interval
            int i = 0;
            for (; i < 4; i++) if (CYCLE_INTERVALS[i] == g_config.cycle_interval_s) break;
            g_config.cycle_interval_s = CYCLE_INTERVALS[(i + 1) % 4];
            break;
        }
        case 2: { // Inactivity pause
            int i = 0;
            for (; i < 3; i++) if (INACTIVITY_VALS[i] == g_config.cycle_inactivity_s) break;
            g_config.cycle_inactivity_s = INACTIVITY_VALS[(i + 1) % 3];
            break;
        }
        case 3: g_config.alert_military = !g_config.alert_military; break;
        case 4: g_config.alert_emergency = !g_config.alert_emergency; break;
        case 5: g_config.trails_enabled = !g_config.trails_enabled; break;
        case 6: g_config.trail_style = g_config.trail_style ? 0 : 1; break;
    }
    storage_save_config(g_config);
    view_changed = true;
}

static void handle_touch_settings(int tx) {
    if (tx < LCD_H_RES / 3) {
        settings_sel = (settings_sel - 1 + NUM_SETTINGS) % NUM_SETTINGS;
        view_changed = true;
    } else if (tx > LCD_H_RES * 2 / 3) {
        settings_sel = (settings_sel + 1) % NUM_SETTINGS;
        view_changed = true;
    } else {
        cycle_view();
    }
}

// ---- Status bar ----

static uint32_t last_status_draw = 0;

static void draw_status_bar(bool force) {
    uint32_t now = millis();
    if (!force && now - last_status_draw < 1000) return;
    last_status_draw = now;

    tft.fillRect(0, 0, LCD_H_RES, STATUS_H, pal->status_bg);
    tft.setTextColor(pal->status, pal->status_bg);
    tft.setTextDatum(TL_DATUM);

    // WiFi status
    const char *net = fetcher_wifi_connected() ? "WiFi" : "---";
    tft.drawString(net, 2, 3, 2);

    // View + Aircraft count
    char buf[24];
    const char *vn = "RDR";
    if (current_view == VIEW_ARRIVALS) vn = "ARR";
    else if (current_view == VIEW_STATS) vn = "STA";
    else if (current_view == VIEW_DETAIL) vn = "DTL";
    else if (current_view == VIEW_LOG) vn = "LOG";
    else if (current_view == VIEW_SETTINGS) vn = "SET";
    snprintf(buf, sizeof(buf), "%s %dAC", vn, aircraft_list.count);
    tft.drawString(buf, 45, 3, 2);

    // Filter indicator (if active)
    int filt = filter_get_active();
    if (filt >= 0 && filt < NUM_FILTERS) {
        tft.setTextColor(filter_defs[filt].color, pal->status_bg);
        tft.drawString(filter_defs[filt].label, 120, 3, 2);
        tft.setTextColor(pal->status, pal->status_bg);
    }

    // Auto-cycle indicator
    if (g_config.cycle_enabled) {
        tft.setTextDatum(TR_DATUM);
        bool paused = (now - last_touch_time) < ((uint32_t)g_config.cycle_inactivity_s * 1000);
        tft.setTextColor(paused ? pal->grid : pal->status, pal->status_bg);
        tft.drawString("AUT", 210, 3, 1);
        tft.setTextColor(pal->status, pal->status_bg);
    }

    // Range
    tft.setTextDatum(TR_DATUM);
    snprintf(buf, sizeof(buf), "%dnm", (int)RANGES[range_idx]);
    tft.drawString(buf, 316, 3, 2);

    // Last update age
    uint32_t age = (now - fetcher_last_update()) / 1000;
    if (fetcher_last_update() > 0) {
        snprintf(buf, sizeof(buf), "%lus", age);
        tft.drawString(buf, 260, 3, 2);
    }

    // Free heap (tiny font center)
    snprintf(buf, sizeof(buf), "%luK", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(buf, 160, 14, 1);
}

// ---- Loading animation ----

static float load_sweep = 0;
static int load_dots = 0;
static uint32_t last_dot_time = 0;

static void draw_loading() {
    const int cx = LCD_H_RES / 2;
    const int cy = LCD_V_RES / 2 - 10;
    const int r = 60;

    // Erase previous sweep line
    float prev = load_sweep - 4.0f;
    if (prev < 0) prev += 360.0f;
    float prad = prev * M_PI / 180.0f;
    tft.drawLine(cx, cy, cx + (int)(r * sinf(prad)), cy - (int)(r * cosf(prad)), pal->bg);

    // Rings + crosshair
    tft.drawCircle(cx, cy, r, pal->ring);
    tft.drawCircle(cx, cy, r * 2 / 3, pal->ring);
    tft.drawCircle(cx, cy, r / 3, pal->ring);
    tft.drawLine(cx - r, cy, cx + r, cy, pal->grid);
    tft.drawLine(cx, cy - r, cx, cy + r, pal->grid);

    // Sweep
    float srad = load_sweep * M_PI / 180.0f;
    tft.drawLine(cx, cy, cx + (int)(r * sinf(srad)), cy - (int)(r * cosf(srad)), pal->sweep);
    tft.fillCircle(cx, cy, 2, pal->text);

    load_sweep += 4.0f;
    if (load_sweep >= 360.0f) load_sweep -= 360.0f;

    // Status text
    int ty = cy + r + 16;
    tft.setTextDatum(MC_DATUM);
    uint32_t now = millis();

    if (!fetcher_wifi_connected()) {
        if (now - last_dot_time > 500) { last_dot_time = now; load_dots = (load_dots + 1) % 4; }
        char msg[20];
        snprintf(msg, sizeof(msg), "WiFi%-3s", load_dots == 1 ? "." : load_dots == 2 ? ".." : load_dots == 3 ? "..." : "");
        tft.setTextColor(pal->text, pal->bg);
        tft.drawString(msg, cx, ty, 2);
    } else if (fetcher_last_update() == 0) {
        if (now - last_dot_time > 500) { last_dot_time = now; load_dots = (load_dots + 1) % 4; }
        char msg[24];
        snprintf(msg, sizeof(msg), "Scanning%-3s", load_dots == 1 ? "." : load_dots == 2 ? ".." : load_dots == 3 ? "..." : "");
        tft.setTextColor(pal->sweep, pal->bg);
        tft.drawString(msg, cx, ty, 2);
    }

    tft.setTextColor(pal->text, pal->bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("ADS-B Radar", cx, cy - r - 20, 4);
}

// ---- Radar view ----

static void draw_radar() {
    if (radar_needs_full_redraw) {
        tft.fillRect(0, RADAR_Y, LCD_H_RES, RADAR_H, pal->bg);
        radar_needs_full_redraw = false;
        prev_blip_count = 0;
        prev_trail_count = 0;
        prev_sweep_angle = -1;
    }

    // Erase old sweep
    if (prev_sweep_angle >= 0) {
        float prad = prev_sweep_angle * M_PI / 180.0f;
        tft.drawLine(RADAR_CX, RADAR_CY,
            RADAR_CX + (int)(RADAR_R * sinf(prad)),
            RADAR_CY - (int)(RADAR_R * cosf(prad)), pal->bg);
    }

    // Erase old trails
    for (int i = 0; i < prev_trail_count; i++) {
        TrailState &t = prev_trails[i];
        for (int j = 1; j < t.count; j++)
            tft.drawLine(t.x[j-1], t.y[j-1], t.x[j], t.y[j], pal->bg);
    }

    // Erase old blips
    for (int i = 0; i < prev_blip_count; i++) {
        BlipState &b = prev_blips[i];
        tft.fillCircle(b.x, b.y, b.r, pal->bg);
        if (b.has_heading) tft.drawLine(b.x, b.y, b.hx, b.hy, pal->bg);
        if (b.has_label) tft.fillRect(b.x + 4, b.y - 12, 56, 18, pal->bg);
    }

    // Redraw static: rings + crosshair
    for (int i = 1; i <= 3; i++) {
        int r = RADAR_R * i / 3;
        tft.drawCircle(RADAR_CX, RADAR_CY, r, pal->ring);
    }
    tft.drawLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_CX + RADAR_R, RADAR_CY, pal->grid);
    tft.drawLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_CX, RADAR_CY + RADAR_R, pal->grid);

    // Compass rose: N/S/E/W at ring edges
    tft.setTextColor(pal->ring, pal->bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("N", RADAR_CX, RADAR_CY - RADAR_R + 6, 1);
    tft.drawString("S", RADAR_CX, RADAR_CY + RADAR_R - 6, 1);
    tft.drawString("E", RADAR_CX + RADAR_R - 6, RADAR_CY, 1);
    tft.drawString("W", RADAR_CX - RADAR_R + 6, RADAR_CY, 1);

    // Sweep line
    float rad = sweep_angle * M_PI / 180.0f;
    int sx = RADAR_CX + (int)(RADAR_R * sinf(rad));
    int sy = RADAR_CY - (int)(RADAR_R * cosf(rad));
    tft.drawLine(RADAR_CX, RADAR_CY, sx, sy, pal->sweep);
    prev_sweep_angle = sweep_angle;

    // Home dot
    tft.fillCircle(RADAR_CX, RADAR_CY, 2, pal->text);

    // Aircraft
    int new_blip_count = 0;
    int new_trail_count = 0;
    if (aircraft_list.lock(pdMS_TO_TICKS(50))) {
        for (int i = 0; i < aircraft_list.count && new_blip_count < MAX_AIRCRAFT; i++) {
            Aircraft &a = aircraft_list.aircraft[i];

            // Apply filter
            if (!aircraft_passes_filter(a)) continue;

            int px, py;
            if (!latlon_to_radar(a.lat, a.lon, px, py)) continue;

            int dx = px - RADAR_CX;
            int dy = py - RADAR_CY;
            if (dx * dx + dy * dy > RADAR_R * RADAR_R) continue;

            uint8_t opacity = compute_aircraft_opacity(a.stale_since, millis());
            if (opacity == 0) continue;

            // Trail
            if (a.trail_count > 1 && new_trail_count < MAX_AIRCRAFT) {
                TrailState &ts = prev_trails[new_trail_count];
                ts.count = 0;
                for (int t = 0; t < a.trail_count && ts.count < TRAIL_LENGTH; t++) {
                    int tx, ty;
                    if (latlon_to_radar(a.trail[t].lat, a.trail[t].lon, tx, ty)) {
                        ts.x[ts.count] = tx;
                        ts.y[ts.count] = ty;
                        ts.count++;
                    }
                }
                for (int t = 1; t < ts.count; t++)
                    tft.drawLine(ts.x[t-1], ts.y[t-1], ts.x[t], ts.y[t], pal->trail);
                new_trail_count++;
            }

            // Sweep-angle fade
            float ac_angle = atan2f((float)(px - RADAR_CX), (float)(RADAR_CY - py)) * 180.0f / M_PI;
            if (ac_angle < 0) ac_angle += 360.0f;
            float behind = sweep_angle - ac_angle;
            if (behind < 0) behind += 360.0f;

            uint16_t fade_color;
            if (a.is_emergency)
                fade_color = (behind < 60) ? pal->blip_emg : pal->fade_emg_dim;
            else if (a.is_military)
                fade_color = (behind < 60) ? pal->blip_mil : pal->fade_mil_dim;
            else if (behind < 60)
                fade_color = pal->fade_bright;
            else if (behind < 180)
                fade_color = pal->fade_med;
            else
                fade_color = pal->fade_dim;

            int blip_r = (a.is_military || a.is_emergency) ? 3 : 2;
            tft.fillCircle(px, py, blip_r, fade_color);

            BlipState &b = prev_blips[new_blip_count];
            b.x = px; b.y = py; b.r = blip_r;
            b.has_heading = false; b.has_label = false;

            // Heading line
            if (a.heading > 0 && !a.on_ground) {
                float hrad = a.heading * M_PI / 180.0f;
                int hx = px + (int)(8 * sinf(hrad));
                int hy = py - (int)(8 * cosf(hrad));
                tft.drawLine(px, py, hx, hy, fade_color);
                b.has_heading = true; b.hx = hx; b.hy = hy;
            }

            // Labels
            if (behind < 180) {
                tft.setTextColor(fade_color, pal->bg);
                tft.setTextDatum(BL_DATUM);
                if (a.callsign[0])
                    tft.drawString(a.callsign, px + 4, py - 2, 1);
                if (behind < 60) {
                    char info[16];
                    if (a.altitude > 0)
                        snprintf(info, sizeof(info), "%d %dk", a.altitude / 100, a.speed);
                    else if (a.on_ground)
                        snprintf(info, sizeof(info), "GND %dk", a.speed);
                    else
                        info[0] = '\0';
                    if (info[0]) {
                        tft.setTextDatum(TL_DATUM);
                        tft.drawString(info, px + 4, py + 2, 1);
                    }
                }
                b.has_label = true;
            }

            new_blip_count++;
        }
        aircraft_list.unlock();
    }
    prev_blip_count = new_blip_count;
    prev_trail_count = new_trail_count;

    // Range labels
    tft.setTextColor(pal->ring, pal->bg);
    tft.setTextDatum(TC_DATUM);
    for (int i = 1; i <= 3; i++) {
        int r = RADAR_R * i / 3;
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", (int)(RANGES[range_idx] * i / 3));
        tft.drawString(buf, RADAR_CX + 2, RADAR_CY - r + 2, 1);
    }
}

// ---- Arrivals board ----

static uint32_t last_arrivals_draw = 0;

static void draw_arrivals() {
    uint32_t now = millis();
    if (!view_changed && now - last_arrivals_draw < 2000) return;

    if (view_changed) {
        tft.fillRect(0, RADAR_Y, LCD_H_RES, RADAR_H, pal->bg);
        view_changed = false;
    }
    last_arrivals_draw = now;

    int y = RADAR_Y + 2;
    tft.setTextColor(pal->hdr, pal->bg);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("CALL", 4, y, 2);
    tft.drawString("ROUTE", 84, y, 2);
    tft.drawString("ALT", 168, y, 2);
    tft.drawString("SPD", 222, y, 2);
    tft.drawString("DIST", 278, y, 2);

    // Sort indicator
    tft.setTextDatum(TR_DATUM);
    char sort_buf[8];
    snprintf(sort_buf, sizeof(sort_buf), "[%s]", SORT_LABELS[arr_sort]);
    tft.drawString(sort_buf, 316, y, 1);

    y += 17;
    tft.drawLine(0, y, LCD_H_RES, y, pal->hdr);
    y += 3;

    struct Entry {
        char call[9];
        char route[10];
        int32_t alt;
        int16_t spd;
        int16_t dist;
        bool mil;
        bool emg;
    };
    static Entry entries[MAX_AIRCRAFT];
    int n = 0;

    if (aircraft_list.lock(pdMS_TO_TICKS(100))) {
        for (int i = 0; i < aircraft_list.count && n < MAX_AIRCRAFT; i++) {
            Aircraft &a = aircraft_list.aircraft[i];
            if (a.lat == 0.0f && a.lon == 0.0f) continue;
            if (compute_aircraft_opacity(a.stale_since, millis()) == 0) continue;
            if (!aircraft_passes_filter(a)) continue;

            Entry &e = entries[n];
            strlcpy(e.call, a.callsign[0] ? a.callsign : a.icao_hex, sizeof(e.call));
            if (a.origin[0] && a.origin[0] != '-' && a.dest[0] && a.dest[0] != '-')
                snprintf(e.route, sizeof(e.route), "%s>%s", a.origin, a.dest);
            else if (a.type_code[0])
                strlcpy(e.route, a.type_code, sizeof(e.route));
            else
                strlcpy(e.route, "---", sizeof(e.route));
            e.alt = a.altitude;
            e.spd = a.speed;
            e.dist = (int16_t)distance_nm(a.lat, a.lon);
            e.mil = a.is_military;
            e.emg = a.is_emergency;
            n++;
        }
        aircraft_list.unlock();
    }

    // Sort by selected mode
    for (int i = 1; i < n; i++) {
        Entry tmp = entries[i];
        int j = i - 1;
        while (j >= 0) {
            bool swap = false;
            switch (arr_sort) {
                case SORT_DIST:  swap = entries[j].dist > tmp.dist; break;
                case SORT_ALT:   swap = entries[j].alt < tmp.alt; break;
                case SORT_SPEED: swap = entries[j].spd < tmp.spd; break;
            }
            if (!swap) break;
            entries[j + 1] = entries[j];
            j--;
        }
        entries[j + 1] = tmp;
    }

    int max_rows = (RADAR_Y + RADAR_H - y) / 16;
    int drawn = 0;
    char buf[16];

    for (int i = 0; i < n && drawn < max_rows; i++) {
        Entry &e = entries[i];
        uint16_t color = pal->blip;
        if (e.emg) color = pal->blip_emg;
        else if (e.mil) color = pal->blip_mil;

        tft.setTextColor(color, pal->bg);
        tft.setTextDatum(TL_DATUM);

        snprintf(buf, sizeof(buf), "%-8s", e.call);
        tft.drawString(buf, 4, y, 2);
        snprintf(buf, sizeof(buf), "%-7s", e.route);
        tft.drawString(buf, 84, y, 2);
        if (e.alt > 0) snprintf(buf, sizeof(buf), "%-4d", e.alt / 100);
        else snprintf(buf, sizeof(buf), "GND ");
        tft.drawString(buf, 168, y, 2);
        snprintf(buf, sizeof(buf), "%-4d", e.spd);
        tft.drawString(buf, 222, y, 2);
        snprintf(buf, sizeof(buf), "%-3d", e.dist);
        tft.drawString(buf, 278, y, 2);

        y += 16;
        drawn++;
    }

    if (y < RADAR_Y + RADAR_H)
        tft.fillRect(0, y, LCD_H_RES, RADAR_Y + RADAR_H - y, pal->bg);
}

// ---- Stats screen ----

static uint32_t last_stats_draw = 0;

static void draw_stats() {
    uint32_t now = millis();
    if (!view_changed && now - last_stats_draw < 2000) return;

    if (view_changed) {
        tft.fillRect(0, RADAR_Y, LCD_H_RES, RADAR_H, pal->bg);
        view_changed = false;
    }
    last_stats_draw = now;

    const FetcherStats *fs = fetcher_get_stats();
    char buf[32];
    int y = RADAR_Y + 4;
    int col1 = 4, col2 = 170;

    auto row = [&](const char *label, const char *value) {
        tft.setTextColor(pal->hdr, pal->bg);
        tft.setTextDatum(TL_DATUM);
        tft.drawString(label, col1, y, 2);
        tft.setTextColor(pal->blip, pal->bg);
        tft.drawString(value, col2, y, 2);
        y += 18;
    };

    uint32_t up = now / 1000;
    snprintf(buf, sizeof(buf), "%luh %lum %lus   ", up / 3600, (up % 3600) / 60, up % 60);
    row("UPTIME", buf);
    row("IP ADDR", fs->ip_addr);
    snprintf(buf, sizeof(buf), "%d dBm   ", WiFi.RSSI());
    row("WIFI RSSI", buf);
    snprintf(buf, sizeof(buf), "%lu ok / %lu fail   ", fs->fetch_ok, fs->fetch_fail);
    row("FETCHES", buf);
    snprintf(buf, sizeof(buf), "%lu ok / %lu fail   ", fs->enrich_ok, fs->enrich_fail);
    row("ENRICH", buf);
    snprintf(buf, sizeof(buf), "%lu ms   ", fs->last_fetch_ms);
    row("FETCH TIME", buf);
    snprintf(buf, sizeof(buf), "%d   ", aircraft_list.count);
    row("AIRCRAFT", buf);

    // Live closest/highest/fastest
    float closest_d = 9999;
    int32_t highest_alt = 0;
    int16_t fastest_spd = 0;
    char closest_call[9] = "---";
    char highest_call[9] = "---";
    char fastest_call[9] = "---";

    if (aircraft_list.lock(pdMS_TO_TICKS(50))) {
        for (int i = 0; i < aircraft_list.count; i++) {
            Aircraft &a = aircraft_list.aircraft[i];
            if (a.lat == 0.0f && a.lon == 0.0f) continue;
            float d = distance_nm(a.lat, a.lon);
            if (d < closest_d) { closest_d = d; strlcpy(closest_call, a.callsign[0] ? a.callsign : a.icao_hex, 9); }
            if (a.altitude > highest_alt) { highest_alt = a.altitude; strlcpy(highest_call, a.callsign[0] ? a.callsign : a.icao_hex, 9); }
            if (a.speed > fastest_spd) { fastest_spd = a.speed; strlcpy(fastest_call, a.callsign[0] ? a.callsign : a.icao_hex, 9); }
        }
        aircraft_list.unlock();
    }

    snprintf(buf, sizeof(buf), "%s %.0fnm   ", closest_call, closest_d < 9999 ? closest_d : 0.0f);
    row("CLOSEST", buf);
    snprintf(buf, sizeof(buf), "%s FL%ld   ", highest_call, highest_alt / 100);
    row("HIGHEST", buf);
    snprintf(buf, sizeof(buf), "%s %dkt   ", fastest_call, fastest_spd);
    row("FASTEST", buf);

    // Closest approach record
    snprintf(buf, sizeof(buf), "%s %.1fnm   ", closest_record.callsign, closest_record.distance_nm < 9999 ? closest_record.distance_nm : 0.0f);
    row("RECORD CLS", buf);

    // Brightness
    snprintf(buf, sizeof(buf), "%d%%   ", (int)(g_config.brightness * 100 / 255));
    row("BRIGHTNESS", buf);
}

// ---- Detail screen ----

static uint32_t last_detail_draw = 0;

static void draw_detail() {
    uint32_t now = millis();
    if (!view_changed && now - last_detail_draw < 1000) return;

    if (view_changed) {
        tft.fillRect(0, RADAR_Y, LCD_H_RES, RADAR_H, pal->bg);
        view_changed = false;
    }
    last_detail_draw = now;

    if (aircraft_list.count == 0) {
        tft.setTextColor(pal->hdr, pal->bg);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("No aircraft", LCD_H_RES / 2, LCD_V_RES / 2, 2);
        return;
    }

    if (detail_idx >= aircraft_list.count) detail_idx = 0;
    if (detail_idx < 0) detail_idx = aircraft_list.count - 1;

    Aircraft a;
    bool got = false;
    if (aircraft_list.lock(pdMS_TO_TICKS(50))) {
        a = aircraft_list.aircraft[detail_idx];
        got = true;
        aircraft_list.unlock();
    }
    if (!got) return;

    tft.fillRect(0, RADAR_Y, LCD_H_RES, RADAR_H, pal->bg);

    int y = RADAR_Y + 2;
    char buf[40];

    // Header
    tft.setTextColor(pal->text, pal->bg);
    tft.setTextDatum(TL_DATUM);
    snprintf(buf, sizeof(buf), "%d/%d", detail_idx + 1, aircraft_list.count);
    tft.drawString(buf, 4, y, 1);

    uint16_t hi_color = a.is_emergency ? pal->blip_emg : a.is_military ? pal->blip_mil : pal->blip;
    tft.setTextColor(hi_color, pal->bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(a.callsign[0] ? a.callsign : a.icao_hex, LCD_H_RES / 2, y + 6, 4);
    y += 28;

    int col1 = 4, col3 = 170;

    auto label_val = [&](int lx, int vx, const char *label, const char *value) {
        tft.setTextColor(pal->hdr, pal->bg);
        tft.setTextDatum(TL_DATUM);
        tft.drawString(label, lx, y, 1);
        tft.setTextColor(pal->blip, pal->bg);
        tft.drawString(value, vx, y, 1);
    };

    label_val(col1, col1 + 30, "HEX", a.icao_hex);
    label_val(col3, col3 + 30, "REG", a.registration[0] ? a.registration : "---");
    y += 12;
    label_val(col1, col1 + 30, "TYP", a.type_code[0] ? a.type_code : "---");
    label_val(col3, col3 + 30, "CAT", a.category[0] ? a.category : "---");
    y += 12;

    tft.setTextColor(pal->hdr, pal->bg);
    tft.drawString("DESC", col1, y, 1);
    tft.setTextColor(pal->blip, pal->bg);
    tft.drawString(a.desc[0] ? a.desc : "---", col1 + 36, y, 1);
    y += 12;

    tft.setTextColor(pal->hdr, pal->bg);
    tft.drawString("OPR", col1, y, 1);
    tft.setTextColor(pal->blip, pal->bg);
    tft.drawString(a.owner_op[0] ? a.owner_op : "---", col1 + 36, y, 1);
    y += 12;

    snprintf(buf, sizeof(buf), "%s > %s",
        (a.origin[0] && a.origin[0] != '-') ? a.origin : "???",
        (a.dest[0] && a.dest[0] != '-') ? a.dest : "???");
    label_val(col1, col1 + 36, "RTE", buf);
    float dist = distance_nm(a.lat, a.lon);
    snprintf(buf, sizeof(buf), "%.0fnm", dist);
    label_val(col3, col3 + 30, "DST", buf);
    y += 14;

    tft.drawLine(0, y, LCD_H_RES, y, pal->grid);
    y += 4;

    snprintf(buf, sizeof(buf), "%ld ft", a.altitude);
    label_val(col1, col1 + 24, "AL", a.on_ground ? "GND" : buf);
    snprintf(buf, sizeof(buf), "%d kt", a.speed);
    label_val(col3, col3 + 24, "GS", buf);
    y += 12;

    snprintf(buf, sizeof(buf), "%d", a.heading);
    label_val(col1, col1 + 24, "HD", buf);
    snprintf(buf, sizeof(buf), "%d fpm", a.vert_rate);
    label_val(col3, col3 + 24, "VS", buf);
    y += 12;

    snprintf(buf, sizeof(buf), "%04d", a.squawk);
    label_val(col1, col1 + 24, "SQ", buf);
    if (a.mach > 0) snprintf(buf, sizeof(buf), "M%.2f", a.mach);
    else strlcpy(buf, "---", sizeof(buf));
    label_val(col3, col3 + 24, "MA", buf);
    y += 12;

    snprintf(buf, sizeof(buf), "%d kt", a.ias);
    label_val(col1, col1 + 30, "IAS", a.ias > 0 ? buf : "---");
    snprintf(buf, sizeof(buf), "%d kt", a.tas);
    label_val(col3, col3 + 30, "TAS", a.tas > 0 ? buf : "---");
    y += 12;

    tft.setTextColor(pal->grid, pal->bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("< prev    tap: next    next >", LCD_H_RES / 2, RADAR_Y + RADAR_H - 8, 1);
}

// ---- Flight log view ----

static uint32_t last_log_draw = 0;

static void draw_log() {
    uint32_t now = millis();
    if (!view_changed && now - last_log_draw < 2000) return;

    if (view_changed) {
        tft.fillRect(0, RADAR_Y, LCD_H_RES, RADAR_H, pal->bg);
        view_changed = false;
    }
    last_log_draw = now;

    int y = RADAR_Y + 2;
    tft.setTextColor(pal->hdr, pal->bg);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("CALL", 4, y, 2);
    tft.drawString("TYPE", 84, y, 2);
    tft.drawString("CLS", 134, y, 2);
    tft.drawString("AGO", 184, y, 2);
    tft.drawString("FLAGS", 254, y, 2);

    char pg_buf[12];
    int total_pages = (log_count + 11) / 12;
    if (total_pages < 1) total_pages = 1;
    if (log_page >= total_pages) log_page = total_pages - 1;
    snprintf(pg_buf, sizeof(pg_buf), "%d/%d", log_page + 1, total_pages);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(pg_buf, 316, y, 1);

    y += 17;
    tft.drawLine(0, y, LCD_H_RES, y, pal->hdr);
    y += 3;

    if (log_count == 0) {
        tft.setTextColor(pal->grid, pal->bg);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("No flights logged", LCD_H_RES / 2, LCD_V_RES / 2, 2);
        return;
    }

    int max_rows = (RADAR_Y + RADAR_H - y) / 14;
    int start = log_page * max_rows;
    int drawn = 0;
    char buf[16];

    // Iterate newest first
    for (int idx = 0; idx < log_count && drawn < max_rows; idx++) {
        int actual_idx = idx + start;
        if (actual_idx >= log_count) break;

        // Newest first: walk backwards from write pointer
        int li = (log_write_idx - 1 - actual_idx + LOG_SIZE) % LOG_SIZE;
        LogEntry &e = flight_log[li];

        uint16_t color = pal->blip;
        if (e.is_emergency) color = pal->blip_emg;
        else if (e.is_military) color = pal->blip_mil;

        tft.setTextColor(color, pal->bg);
        tft.setTextDatum(TL_DATUM);

        snprintf(buf, sizeof(buf), "%-8s", e.callsign);
        tft.drawString(buf, 4, y, 1);

        snprintf(buf, sizeof(buf), "%-4s", e.type_code);
        tft.drawString(buf, 84, y, 1);

        if (e.closest_nm < 9999)
            snprintf(buf, sizeof(buf), "%.0f", e.closest_nm);
        else
            strlcpy(buf, "---", sizeof(buf));
        tft.drawString(buf, 134, y, 1);

        // Time ago
        uint32_t ago_s = (now - e.first_seen) / 1000;
        if (ago_s < 60) snprintf(buf, sizeof(buf), "%lus", ago_s);
        else if (ago_s < 3600) snprintf(buf, sizeof(buf), "%lum", ago_s / 60);
        else snprintf(buf, sizeof(buf), "%luh", ago_s / 3600);
        tft.drawString(buf, 184, y, 1);

        // Flags
        const char *flag = "";
        if (e.is_emergency) flag = "EMG";
        else if (e.is_military) flag = "MIL";
        tft.drawString(flag, 254, y, 1);

        y += 14;
        drawn++;
    }

    if (y < RADAR_Y + RADAR_H)
        tft.fillRect(0, y, LCD_H_RES, RADAR_Y + RADAR_H - y, pal->bg);
}

// ---- Settings screen ----

static uint32_t last_settings_draw = 0;

static void draw_settings() {
    uint32_t now = millis();
    if (!view_changed && now - last_settings_draw < 500) return;

    if (view_changed) {
        tft.fillRect(0, RADAR_Y, LCD_H_RES, RADAR_H, pal->bg);
        view_changed = false;
    }
    last_settings_draw = now;

    int y = RADAR_Y + 2;
    tft.setTextColor(pal->hdr, pal->bg);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("SETTINGS", 4, y, 2);

    char pg[8];
    snprintf(pg, sizeof(pg), "%d/%d", settings_sel + 1, NUM_SETTINGS);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(pg, 316, y, 1);

    y += 17;
    tft.drawLine(0, y, LCD_H_RES, y, pal->hdr);
    y += 4;

    char buf[16];
    for (int i = 0; i < NUM_SETTINGS; i++) {
        bool sel = (i == settings_sel);
        uint16_t label_color = sel ? pal->blip : pal->hdr;
        uint16_t value_color = sel ? pal->blip : pal->text;
        uint16_t row_bg = pal->bg;

        // Highlight selected row
        if (sel) {
            tft.fillRect(0, y, LCD_H_RES, 18, pal->grid);
            row_bg = pal->grid;
        } else {
            tft.fillRect(0, y, LCD_H_RES, 18, pal->bg);
        }

        tft.setTextColor(label_color, row_bg);
        tft.setTextDatum(TL_DATUM);
        tft.drawString(SETTINGS_LABELS[i], 8, y + 2, 2);

        // Value
        switch (i) {
            case 0: strlcpy(buf, g_config.cycle_enabled ? "ON" : "OFF", sizeof(buf)); break;
            case 1: snprintf(buf, sizeof(buf), "%ds", g_config.cycle_interval_s); break;
            case 2: snprintf(buf, sizeof(buf), "%ds", g_config.cycle_inactivity_s); break;
            case 3: strlcpy(buf, g_config.alert_military ? "ON" : "OFF", sizeof(buf)); break;
            case 4: strlcpy(buf, g_config.alert_emergency ? "ON" : "OFF", sizeof(buf)); break;
            case 5: strlcpy(buf, g_config.trails_enabled ? "ON" : "OFF", sizeof(buf)); break;
            case 6: strlcpy(buf, g_config.trail_style ? "DOTS" : "LINE", sizeof(buf)); break;
        }

        tft.setTextColor(value_color, row_bg);
        tft.setTextDatum(TR_DATUM);
        tft.drawString(buf, 308, y + 2, 2);

        y += 22;
    }

    // Footer hint
    tft.setTextColor(pal->grid, pal->bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("< prev   tap: next   hold: change >", LCD_H_RES / 2, RADAR_Y + RADAR_H - 8, 1);
}

// ---- Alert overlay ----

static void check_alerts() {
    if (!g_config.alert_military && !g_config.alert_emergency) return;

    if (aircraft_list.lock(pdMS_TO_TICKS(30))) {
        for (int i = 0; i < aircraft_list.count; i++) {
            Aircraft &a = aircraft_list.aircraft[i];

            bool is_alert = false;
            const char *type_str = "";
            uint16_t color = pal->blip;

            if (g_config.alert_military && a.is_military) {
                is_alert = true; type_str = "MIL"; color = pal->blip_mil;
            } else if (g_config.alert_emergency && a.is_emergency) {
                is_alert = true; type_str = "EMG"; color = pal->blip_emg;
            }

            if (!is_alert) continue;

            // Dedup check
            uint32_t h = hash_icao(a.icao_hex);
            bool dup = false;
            for (int d = 0; d < ALERT_DEDUP_SIZE; d++) {
                if (alerted_icaos[d] == h) { dup = true; break; }
            }
            if (dup) continue;

            // New alert
            alerted_icaos[alerted_idx] = h;
            alerted_idx = (alerted_idx + 1) % ALERT_DEDUP_SIZE;

            alert.active = true;
            alert.start_time = millis();
            alert.color = color;
            snprintf(alert.message, sizeof(alert.message), "%s %s", type_str, a.callsign[0] ? a.callsign : a.icao_hex);
            break;  // one alert at a time
        }
        aircraft_list.unlock();
    }
}

static void draw_alert_overlay() {
    if (!alert.active) return;

    uint32_t elapsed = millis() - alert.start_time;
    if (elapsed > ALERT_DURATION_MS) {
        alert.active = false;
        radar_needs_full_redraw = true;
        view_changed = true;
        return;
    }

    // Flashing border (toggle every 250ms)
    bool on = ((elapsed / 250) % 2) == 0;
    uint16_t border_color = on ? alert.color : pal->bg;

    tft.drawRect(0, STATUS_H, LCD_H_RES, RADAR_H, border_color);
    tft.drawRect(1, STATUS_H + 1, LCD_H_RES - 2, RADAR_H - 2, border_color);

    // Banner at top of view area
    if (on) {
        tft.fillRect(2, STATUS_H + 2, LCD_H_RES - 4, 16, alert.color);
        tft.setTextColor(pal->bg, alert.color);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(alert.message, LCD_H_RES / 2, STATUS_H + 10, 2);
    }
}

// ---- Closest approach + flight log update ----

static uint32_t last_once_per_sec = 0;

static void update_once_per_sec() {
    uint32_t now = millis();
    if (now - last_once_per_sec < 1000) return;
    last_once_per_sec = now;

    check_alerts();

    if (!aircraft_list.lock(pdMS_TO_TICKS(30))) return;

    for (int i = 0; i < aircraft_list.count; i++) {
        Aircraft &a = aircraft_list.aircraft[i];
        if (a.lat == 0.0f && a.lon == 0.0f) continue;
        if (compute_aircraft_opacity(a.stale_since, now) == 0) continue;

        float d = distance_nm(a.lat, a.lon);

        // Update closest approach record
        if (d < closest_record.distance_nm) {
            closest_record.distance_nm = d;
            closest_record.altitude = a.altitude;
            closest_record.timestamp = now;
            strlcpy(closest_record.callsign, a.callsign[0] ? a.callsign : a.icao_hex, sizeof(closest_record.callsign));
            strlcpy(closest_record.type_code, a.type_code, sizeof(closest_record.type_code));
        }

        // Flight log: check if already logged
        uint32_t h = hash_icao(a.icao_hex);
        bool found = false;
        for (int j = 0; j < log_count; j++) {
            int li = (log_write_idx - 1 - j + LOG_SIZE) % LOG_SIZE;
            if (flight_log[li].icao_hash == h) {
                // Update closest distance
                if (d < flight_log[li].closest_nm)
                    flight_log[li].closest_nm = d;
                found = true;
                break;
            }
        }
        if (!found) {
            LogEntry &e = flight_log[log_write_idx];
            e.icao_hash = h;
            strlcpy(e.callsign, a.callsign[0] ? a.callsign : a.icao_hex, sizeof(e.callsign));
            strlcpy(e.type_code, a.type_code, sizeof(e.type_code));
            e.first_seen = now;
            e.closest_nm = d;
            e.is_military = a.is_military;
            e.is_emergency = a.is_emergency;
            log_write_idx = (log_write_idx + 1) % LOG_SIZE;
            if (log_count < LOG_SIZE) log_count++;
        }
    }
    aircraft_list.unlock();
}

// ---- Auto-cycle ----

static void auto_cycle() {
    if (!g_config.cycle_enabled) return;
    if (current_view == VIEW_LOADING || current_view == VIEW_DETAIL || current_view == VIEW_SETTINGS) return;

    uint32_t now = millis();

    // Paused if recent touch
    if ((now - last_touch_time) < ((uint32_t)g_config.cycle_inactivity_s * 1000)) return;

    // Dwell time depends on view
    uint32_t dwell_ms;
    switch (current_view) {
        case VIEW_STATS: dwell_ms = 5000; break;
        case VIEW_LOG:   dwell_ms = 10000; break;
        default:         dwell_ms = (uint32_t)g_config.cycle_interval_s * 1000; break;
    }

    if ((now - last_cycle_time) >= dwell_ms) {
        last_cycle_time = now;
        // Cycle through: RADAR -> ARRIVALS -> STATS -> LOG -> RADAR (skip DETAIL)
        switch (current_view) {
            case VIEW_RADAR:    current_view = VIEW_ARRIVALS; view_changed = true; break;
            case VIEW_ARRIVALS: current_view = VIEW_STATS;    view_changed = true; break;
            case VIEW_STATS:    current_view = VIEW_LOG;      view_changed = true; break;
            case VIEW_LOG:      current_view = VIEW_RADAR;    radar_needs_full_redraw = true; break;
            default: break;
        }
        draw_status_bar(true);
    }
}

// ---- Setup ----

void setup() {
    Serial.begin(115200);
    Serial.println("ADS-B CYD Radar starting...");

    // Backlight via PWM
    ledcAttach(TFT_BL, 5000, 8);  // 5kHz, 8-bit resolution

    // Display
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    // Touch
    touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    pinMode(XPT2046_IRQ, INPUT);
    pinMode(XPT2046_CS, OUTPUT);
    digitalWrite(XPT2046_CS, HIGH);

    // Data layer
    error_log_init();
    g_config = storage_load_config();
    aircraft_list.init();
    enrichment_init();
    fetcher_init(&aircraft_list);

    // Apply persisted settings
    pal = g_config.night_mode ? &PALETTE_NIGHT : &PALETTE_GREEN;
    apply_brightness();

    uint32_t now = millis();
    last_touch_time = now;
    last_cycle_time = now;

    Serial.printf("Aircraft array: %d x %d = %d bytes\n",
        MAX_AIRCRAFT, (int)sizeof(Aircraft), MAX_AIRCRAFT * (int)sizeof(Aircraft));
    Serial.printf("Free heap: %lu\n",
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

// ---- Main loop ----

static uint32_t last_draw = 0;
static bool touch_was_down = false;

void loop() {
    uint32_t now = millis();

    // Touch handling with long-press detection
    int tx, ty;
    if (getTouchPoint(tx, ty)) {
        if (!touch_was_down) {
            touch_was_down = true;
            touch_down_time = now;
            long_press_fired = false;
            saved_tx = tx;
            last_touch_time = now;
            last_cycle_time = now;
        } else {
            saved_tx = tx;  // track latest position
            if (!long_press_fired && (now - touch_down_time) >= LONG_PRESS_MS) {
                long_press_fired = true;
                if (saved_tx >= LCD_H_RES / 3 && saved_tx <= LCD_H_RES * 2 / 3) {
                    if (current_view == VIEW_STATS)
                        toggle_night_mode();
                    else if (current_view == VIEW_SETTINGS)
                        settings_adjust_selected();
                }
            }
        }
    } else {
        if (touch_was_down && !long_press_fired) {
            switch (current_view) {
                case VIEW_RADAR:    handle_touch_radar(saved_tx); break;
                case VIEW_ARRIVALS: handle_touch_arrivals(saved_tx); break;
                case VIEW_STATS:    handle_touch_stats(saved_tx, false); break;
                case VIEW_DETAIL:   handle_touch_detail(saved_tx); break;
                case VIEW_LOG:      handle_touch_log(saved_tx); break;
                case VIEW_SETTINGS: handle_touch_settings(saved_tx); break;
                default: break;
            }
            draw_status_bar(true);
        }
        touch_was_down = false;
    }

    // Once-per-second updates (alerts, log, closest)
    update_once_per_sec();

    // Auto-cycle
    auto_cycle();

    // Redraw at ~15fps
    if (now - last_draw >= 66) {
        last_draw = now;

        if (current_view == VIEW_LOADING) {
            draw_loading();
            if (fetcher_last_update() > 0) {
                current_view = VIEW_RADAR;
                tft.fillScreen(pal->bg);
                radar_needs_full_redraw = true;
                draw_status_bar(true);
            }
        } else {
            sweep_angle += SWEEP_SPEED;
            if (sweep_angle >= 360.0f) sweep_angle -= 360.0f;

            draw_status_bar(false);
            switch (current_view) {
                case VIEW_RADAR:    draw_radar(); break;
                case VIEW_ARRIVALS: draw_arrivals(); break;
                case VIEW_STATS:    draw_stats(); break;
                case VIEW_DETAIL:   draw_detail(); break;
                case VIEW_LOG:      draw_log(); break;
                case VIEW_SETTINGS: draw_settings(); break;
                default: break;
            }

            // Alert overlay on top
            draw_alert_overlay();
        }
    }

    enrichment_poll();
    delay(5);
}
