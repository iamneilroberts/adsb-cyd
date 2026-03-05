#pragma once
#include <cstdint>

struct AircraftEnrichment {
    char photo_url[256];
    char photo_photographer[48];
    char airline[32];
    char origin_airport[48];      // Full name: "John F Kennedy International Airport"
    char destination_airport[48]; // Full name: "Los Angeles International Airport"
    char manufacturer[32];
    char model[48];
    char owner[48];
    char registered_country[24];
    char engine_type[24];
    uint8_t engine_count;
    uint16_t year_built;
    bool loaded;
    bool loading;
};

// Initialize enrichment system
void enrichment_init();

// Poll for deferred callbacks (call from main loop)
void enrichment_poll();

// Fetch enrichment data in background. Calls callback progressively as data arrives.
void enrichment_fetch(const char *icao_hex, const char *registration,
                      const char *callsign,
                      void (*callback)(AircraftEnrichment *data));

// Get cached enrichment (returns nullptr if not yet fetched)
AircraftEnrichment *enrichment_get_cached(const char *icao_hex);
