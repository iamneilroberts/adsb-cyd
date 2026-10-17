# ADS-B Radar — CYD Edition

A real-time aircraft radar display for the **ESP32-2432S028** (aka "Cheap Yellow Display" / CYD). Tracks aircraft using the free adsb.lol API and displays them on a 320x240 TFT screen with touch interaction.

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
- **Large touch hot zones** — status strip plus left/center/right thirds, outlined for 5 seconds at boot
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
- **USB connector varies by board revision.** The original boards have a Micro USB port; newer revisions have a **USB-C** port instead of, or in addition to, the Micro USB one. Either connector works for power, flashing and the serial monitor. Some revisions also use a different USB-to-serial chip (CH340 or CP2102), so install the matching driver (see Setup step 2).

### Board layout

![CYD board back](images/ESP32-Cheap-Yellow-Display-CYD-Board-ESP32-2432S028R-back-labeled.jpg)

Key components (viewing the back of the board):
- **Micro USB** (left side) — for power, programming, and serial monitor. Newer revisions have a **USB-C** port (and some keep both).
- **BOOT button** — near the center of the board, next to the speaker
- **RST button** — top-right corner
- **ESP-WROOM-32** — the metal-shielded module on the right

## Setup

### 1. Install PlatformIO

Install [VS Code](https://code.visualstudio.com/) and the [PlatformIO extension](https://platformio.org/install/ide?install=vscode), or install the CLI:

```bash
pip install platformio
```

### 2. Install USB drivers

Your CYD has a **CH340** USB-to-serial chip for programming. Some board variants also include a **CP2102** chip, but the CH340 is the one used for flashing.

- **CH340** — [download from manufacturer](http://www.wch-ic.com/downloads/CH341SER_ZIP.html) (Windows/Mac/Linux)
- **CP2102** (if your board has one) — [download from Silicon Labs](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)

Most Linux distros include both drivers already. On Windows/Mac, install the driver **before** plugging in the board.

**Linux only:** Add yourself to the `dialout` group so you can access the serial port:

```bash
sudo usermod -aG dialout $USER
```

Then **log out and back in** (or reboot) for this to take effect.

### 3. Clone this repo

```bash
git clone https://github.com/iamneilroberts/adsb-cyd.git
cd adsb-cyd
```

### 4. Configure your location

Edit `src/config.h` and set your WiFi credentials and home coordinates:

```c
#define WIFI_SSID "YourWiFiName"
#define WIFI_PASS "YourWiFiPassword"

#define HOME_LAT 30.6905    // Your latitude
#define HOME_LON -88.1632   // Your longitude
```

You can find your coordinates at [latlong.net](https://www.latlong.net/).

### 5. Connect the CYD

Plug in the CYD via the **Micro USB** port (left side of the board, when viewing from the back), or the **USB-C** port if your board revision has one. **Use a data cable, not a charge-only cable** — if the board isn't detected, the cable is the most likely cause.

Verify the board is detected:

- **Linux:** `ls /dev/ttyUSB*` — you should see `/dev/ttyUSB0`
- **Mac:** `ls /dev/cu.usbserial*` or `ls /dev/cu.wchusbserial*`
- **Windows:** Open Device Manager → Ports — look for "USB-SERIAL CH340" or "CP210x"

### 6. Know your buttons

Refer to the [board layout photo](#board-layout) above. The two buttons on the back are:

- **BOOT** — closer to the center of the board (near the speaker)
- **RST** (Reset) — in the top-right corner

You may need these during flashing (see step 7).

### 7. Build and flash

```bash
pio run -e cyd -t upload
```

If the upload fails with a connection error, try holding the **BOOT** button:

1. Hold **BOOT**
2. While holding BOOT, press and release **RST**
3. Release **BOOT**
4. Run the upload command within a few seconds

If the upload fails because it can't find the port, specify it explicitly:

```bash
# Linux
pio run -e cyd -t upload --upload-port /dev/ttyUSB0

# Mac
pio run -e cyd -t upload --upload-port /dev/cu.usbserial-XYZ

# Windows
pio run -e cyd -t upload --upload-port COM3
```

### 8. Verify it works

After flashing, the CYD will reboot and show a startup screen. It will:

1. Connect to your WiFi (the screen shows connection status)
2. Start fetching aircraft data
3. Display the radar view with any aircraft in range

If the screen stays white or blank, double-check your WiFi credentials in `src/config.h` and re-flash.

### 9. Monitor serial output (optional)

To see debug output and connection info:

```bash
pio device monitor
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Board not detected (no `/dev/ttyUSB*` or COM port) | Try a different USB cable — charge-only cables won't work. Install the CH340 or CP2102 driver (see step 2). |
| "Permission denied" on Linux | Run `sudo usermod -aG dialout $USER`, then log out and back in. |
| Upload fails with "connection timeout" or "invalid head of packet" | Put the board in download mode: hold **BOOT**, press and release **RST**, release **BOOT**, then upload within a few seconds. |
| Upload still fails after BOOT/RST | Try a lower baud rate: add `upload_speed = 115200` to `platformio.ini`. Also try a different USB cable. |
| "Invalid head of packet" every time | Some CYD boards have two serial chips (CH340 + CP2102). The CH340 is the programming port. If you see two `/dev/ttyUSB*` devices, try the other one. On Linux, run `lsusb` to identify which is which. |
| Screen is white/blank after flash | Check WiFi credentials in `src/config.h`. Open serial monitor (`pio device monitor`) to see error messages. |
| No aircraft showing | Verify your coordinates are correct. Make sure you're within range of aircraft (25nm default, see below). The free API may have brief outages — wait a minute and check again. |

## Touch Controls

The screen uses large fixed hot zones, sized for a resistive touchscreen: a **status strip** across the top (28 px) and, below it, **left**, **center** and **right** thirds of the screen. Tap for a short press, or hold for 1 second for a long press.

- **Status strip (top)** — always goes to the next view, in every view.
- **Left / center / right thirds** — the per-view actions in the table below.

For 5 seconds after boot, once the loading screen finishes, the zones are outlined in yellow and labeled on the radar view so you can see where to tap. This also works as a touch-calibration check: if a tap lands in the wrong box, adjust the `TOUCH_*` values in `src/config.h`.

A tap's position is latched just after contact and a release needs 40 ms without touch, which filters out the noisy readings a resistive panel gives as a finger lifts.

| View | Left tap | Center tap | Center HOLD (1s) | Right tap |
|------|----------|------------|------------------|-----------|
| **Radar** | Cycle filter (ALL → COM → MIL → EMG → HELI → FAST → SLOW → ODD) | Next view | — | Cycle range (25 → 15 → 10 → 5nm) |
| **Arrivals** | Bottom button strip, left: cycle sort (DST → ALT → SPD) | Tap an aircraft row anywhere across its width → Detail view (use the status strip for next view) | — | Bottom button strip, right: cycle range |
| **Stats** | Brightness down (-12%) | Next view | Toggle night mode (green/amber) | Brightness up (+12%) |
| **Log** | Previous page | Next view | — | Next page |
| **Settings** | Previous setting | Next view | Toggle/change selected setting | Next setting |
| **Detail** | Previous aircraft | Back to Radar | — | Next aircraft |

**Auto-cycle indicator (status bar):** `AUTO` = running; yellow `PAUSED 42s` = paused after a touch, with the seconds left before it resumes; yellow `AUTO HOLD` = on the Detail or Settings view, which never auto-cycle. Nothing is shown when auto-cycle is off in Settings.

**View cycle:** Radar → Arrivals → Stats → Log → Settings → Radar

**Detail view** is not part of the normal cycle — tap an aircraft row in the Arrivals list to open it.

## Settings Screen

Navigate to the settings screen by tapping through views (Radar → Arrivals → Stats → Log → **Settings**).

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
- **Enrichment data:** [adsbdb.com](https://www.adsbdb.com) (callsign/aircraft lookups), [planespotters.net](https://www.planespotters.net) (photos)

### Fetch radius and polling

`src/config.h` sets how much data is requested:

```c
#define ADSB_RADIUS_NM 25            // fetch radius
#define ADSB_POLL_INTERVAL_MS 15000  // time between fetches
```

- **Radius is limited by RAM.** The whole response is buffered before parsing, and the CYD has no PSRAM. Around 25nm in busy airspace is ~30 KB; 35nm was already ~56 KB, bigger than the largest free heap block (~43 KB), so the buffer allocation failed and nothing was parsed (`JSON error: EmptyInput`). The radar ranges (25/15/10/5nm) are capped at the fetch radius for this reason. Quiet areas can use a larger radius.
- **Rate limits.** The free APIs return HTTP 429 if polled too often. Keep the interval at 10s or more.
- **User-Agent.** Requests identify as `adsb-cyd-radar/1.1 (xxxxxx)`, where `xxxxxx` comes from the chip's MAC address. It is built once and stable per device, which keeps the API operators' view of your traffic clean and helps with bot filters that reject the default ESP32 User-Agent.

## Power

The CYD has no battery charger. If you add a battery, use a charger/boost module with power-path (load-sharing) charging, such as an IP5306-based board or an Adafruit PowerBoost. Simple TP4056 + boost setups can drop the 5V rail when USB is plugged or unplugged and reset the board; a 470–1000 µF capacitor across 5V and GND helps. A `Brownout detector was triggered` message in the serial monitor confirms this.

## Flash Usage

```
RAM:   19.9% (64KB / 320KB)
Flash: 88.7% (1.16MB / 1.31MB) — ~148KB free
```

## License

MIT
