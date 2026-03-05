#pragma once
#include "../data/aircraft.h"

#define FILT_NONE     -1
#define FILT_AIRLINE   0
#define FILT_MILITARY  1
#define FILT_EMERGENCY 2
#define FILT_HELI      3
#define FILT_FAST      4
#define FILT_SLOW      5
#define FILT_ODDBALL   6
#define NUM_FILTERS    7

struct FilterDef {
    const char *label;
    uint16_t color;  // TFT_eSPI color565
};

extern const FilterDef filter_defs[NUM_FILTERS];

int  filter_get_active();
void filter_set_active(int idx);
void filter_cycle();  // cycle through ALL -> COM -> MIL -> ... -> ALL

bool aircraft_passes_filter(const Aircraft &ac);
bool is_airline_callsign(const char *cs);
bool is_heli_type(const char *t);
