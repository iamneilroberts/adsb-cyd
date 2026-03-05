#include "filters.h"
#include <TFT_eSPI.h>
#include <cstring>

static uint16_t c565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

const FilterDef filter_defs[NUM_FILTERS] = {
    {"COM",  c565(0x44, 0x88, 0xff)},
    {"MIL",  c565(0xff, 0xaa, 0x00)},
    {"EMG",  c565(0xff, 0x33, 0x33)},
    {"HELI", c565(0x44, 0xdd, 0xaa)},
    {"FAST", c565(0xff, 0x66, 0xcc)},
    {"SLOW", c565(0x88, 0xaa, 0xcc)},
    {"ODD",  c565(0xcc, 0x88, 0xff)},
};

static int _active_filter = FILT_NONE;

int filter_get_active() { return _active_filter; }
void filter_set_active(int idx) { _active_filter = idx; }

void filter_cycle() {
    _active_filter++;
    if (_active_filter >= NUM_FILTERS) _active_filter = FILT_NONE;
}

bool is_airline_callsign(const char *cs) {
    return cs[0] >= 'A' && cs[0] <= 'Z' &&
           cs[1] >= 'A' && cs[1] <= 'Z' &&
           cs[2] >= 'A' && cs[2] <= 'Z' &&
           cs[3] >= '0' && cs[3] <= '9';
}

bool is_heli_type(const char *t) {
    static const char *heli_types[] = {
        "R22", "R44", "R66", "EC35", "EC45", "EC55",
        "A109", "A139", "A169", "B06", "B212", "B412",
        "S76", "S92", "B407", "B429", "B505",
        "H135", "H145", "H160", "H175", "H225",
        "AS50", "AS55", "AS65", "MD52", "MD60",
        "NH90", "CH47", "V22", "UH1", "BK17",
        nullptr
    };
    for (int i = 0; heli_types[i]; i++) {
        if (strcmp(t, heli_types[i]) == 0) return true;
    }
    return false;
}

bool aircraft_passes_filter(const Aircraft &ac) {
    if (_active_filter == FILT_NONE) return true;

    switch (_active_filter) {
        case FILT_AIRLINE:
            if (is_airline_callsign(ac.callsign)) return true;
            if (ac.category[0] == 'A' && ac.category[1] >= '3') return true;
            return false;
        case FILT_MILITARY:
            return ac.is_military;
        case FILT_EMERGENCY:
            return ac.is_emergency;
        case FILT_HELI:
            if (ac.category[0] == 'A' && ac.category[1] == '7') return true;
            if (ac.type_code[0] && is_heli_type(ac.type_code)) return true;
            return false;
        case FILT_FAST:
            return ac.speed > 300 && !ac.on_ground;
        case FILT_SLOW:
            return ac.speed > 0 && ac.speed < 100 && !ac.on_ground;
        case FILT_ODDBALL:
            if (ac.category[0] == 'B') return true;
            if (ac.registration[0] == 'N' && !is_airline_callsign(ac.callsign)) {
                if (ac.category[0] == 'A' && (ac.category[1] == '1' || ac.category[1] == '2')) return true;
                if (!ac.category[0] && ac.speed < 200) return true;
            }
            return false;
    }
    return true;
}
