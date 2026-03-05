# ADS-B Radar — CYD Edition

A real-time aircraft radar display for the **ESP32-2432S028** (aka "Cheap Yellow Display" / CYD). Tracks aircraft using the ADS-B Exchange API and displays them on a 320x240 TFT screen with touch interaction.

No PSRAM required. No LVGL. Just direct TFT_eSPI rendering on a $15 board.

## Features

- **Radar view** — sweeping radar with aircraft blips, trails, heading lines, and sweep-based fading
- **Arrivals board** — tabular list with callsign, route, altitude, speed, distance
- **Stats dashboard** — uptime, WiFi, fetch stats, closest/highest/fastest aircraft
- **Flight log** — circular buffer of 50 recently seen aircraft
- **Detail view** — full aircraft info (type, operator, route, squawk, IAS/TAS/Mach)
- **Settings screen** — configure auto-cycle, alerts, trails via touch (persisted to NVS)
- **Compass rose** — N/S/E/W labels on radar
- **Auto-cycle** — automatically rotates views, pauses on touch
- **Brightness control** — PWM backlight, adjustable from stats view
- **Night mode** — amber/red palette, long-press to toggle
- **Filters** — cycle through ALL/COM/MIL/EMG/HELI/FAST/SLOW/ODD
- **Sort modes** — arrivals sortable by distance, altitude, or speed
- **Mil/Emergency alerts** — flashing border + banner on military/emergency aircraft
- **Closest approach record** — tracks the nearest aircraft ever seen

## Hardware

- **ESP32-2432S028** (ESP32-WROOM-32 with 2.8" ILI9341 320x240 TFT + XPT2046 touch)
- Available on AliExpress/Amazon for ~$15
- No additional wiring needed — everything is on-board

## Setup

### 1. Install PlatformIO

Install [VS Code](https://code.visualstudio.com/) and the [PlatformIO extension](https://platformio.org/install/ide?install=vscode), or install the CLI:

```bash
pip install platformio
```

### 2. Clone this repo

```bash
git clone https://github.com/iamneilroberts/adsb-cyd.git
cd adsb-cyd
```

### 3. Configure your location

Edit `src/config.h` and set your home coordinates and WiFi credentials:

```c
#define WIFI_SSID "YourWiFiName"
#define WIFI_PASS "YourWiFiPassword"

#define HOME_LAT 30.6905    // Your latitude
#define HOME_LON -88.1632   // Your longitude
```

You can find your coordinates at [latlong.net](https://www.latlong.net/).

### 4. Build

```bash
pio run -e cyd
```

### 5. Flash

Plug in the CYD via USB. It usually shows up as `/dev/ttyUSB0` (Linux) or `COM3` (Windows).

```bash
pio run -e cyd -t upload
```

If you have multiple USB devices, specify the port:

```bash
pio run -e cyd -t upload --upload-port /dev/ttyUSB0
```

**Windows:** Replace `/dev/ttyUSB0` with your COM port (e.g., `COM3`). Check Device Manager under "Ports" if unsure.

**First time on Linux?** You may need to add yourself to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```
Then log out and back in.

### 6. Monitor serial output (optional)

```bash
pio device monitor -e cyd
```

## Touch Controls

The screen is divided into three vertical zones: **left third**, **middle third**, **right third**.

| View | Left tap | Middle tap | Right tap |
|------|----------|------------|-----------|
| **Radar** | Cycle filter | Next view | Cycle range |
| **Arrivals** | Cycle sort | Next view | Cycle range |
| **Stats** | Brightness - | Next view | Brightness + |
| **Log** | Page up | Next view | Page down |
| **Settings** | Prev setting | Next view | Next setting |
| **Detail** | Prev aircraft | Next view | Next aircraft |

**Long press (hold 1s) on middle:**
- **Stats view** — toggles night mode (amber/red palette)
- **Settings view** — changes the highlighted setting

## Settings Screen

Navigate to the settings screen by tapping through views (RADAR → ARRIVALS → STATS → LOG → **SETTINGS**).

- Tap **left/right** to highlight a setting
- **Long-press middle** (hold for 1 second) to toggle or cycle the value
- Changes save to flash immediately and persist across reboots

Available settings:
| Setting | Values |
|---------|--------|
| Auto Cycle | ON / OFF |
| Cycle Interval | 15s / 30s / 60s / 90s |
| Inactivity Pause | 30s / 60s / 120s |
| Alert Military | ON / OFF |
| Alert Emergency | ON / OFF |
| Trails | ON / OFF |
| Trail Style | LINE / DOTS |

## Data Sources

- **Aircraft positions:** [api.adsb.lol](https://api.adsb.lol) (free, no API key)
- **Enrichment data:** [adsbdb.com](https://www.adsbdb.com) (callsign/aircraft lookups)

## Flash Usage

```
RAM:   19.8% (64KB / 320KB)
Flash: 88.0% (1.15MB / 1.31MB) — ~157KB free
```

## License

MIT
