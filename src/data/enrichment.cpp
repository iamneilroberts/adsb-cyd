#include "enrichment.h"
#include "http_mutex.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <cstring>

#define MAX_CACHE 10

static AircraftEnrichment _cache[MAX_CACHE];
static char _cache_keys[MAX_CACHE][7];
static int _cache_count = 0;

static void (*_pending_callback)(AircraftEnrichment *) = nullptr;
static volatile bool _task_running = false;

static volatile AircraftEnrichment *_deferred_entry = nullptr;
static volatile bool _deferred_ready = false;

struct EnrichParams {
    char icao_hex[7];
    char callsign[9];
};

AircraftEnrichment *enrichment_get_cached(const char *icao_hex) {
    for (int i = 0; i < _cache_count; i++) {
        if (strcmp(_cache_keys[i], icao_hex) == 0 && _cache[i].loaded) {
            return &_cache[i];
        }
    }
    return nullptr;
}

static AircraftEnrichment *get_or_create_cache_entry(const char *icao_hex) {
    for (int i = 0; i < _cache_count; i++) {
        if (strcmp(_cache_keys[i], icao_hex) == 0) return &_cache[i];
    }
    int idx = _cache_count < MAX_CACHE ? _cache_count++ : 0;
    memset(&_cache[idx], 0, sizeof(AircraftEnrichment));
    strlcpy(_cache_keys[idx], icao_hex, 7);
    return &_cache[idx];
}

static void notify_callback(AircraftEnrichment *entry) {
    _deferred_entry = entry;
    _deferred_ready = true;
}

static void fetch_task(void *param) {
    _task_running = true;
    EnrichParams *params = (EnrichParams *)param;
    AircraftEnrichment *entry = get_or_create_cache_entry(params->icao_hex);
    entry->loading = true;

    // Stage 1: Airport names
    if (params->callsign[0] && http_mutex_acquire(pdMS_TO_TICKS(8000))) {
        char url[128];
        snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/callsign/%s", params->callsign);
        WiFiClientSecure client;
        client.setInsecure();
        client.setHandshakeTimeout(5);
        HTTPClient http;
        http.begin(client, url);
        http.setTimeout(5000);
        if (http.GET() == HTTP_CODE_OK) {
            String payload = http.getString();
            JsonDocument doc;
            if (!deserializeJson(doc, payload)) {
                JsonObject route = doc["response"]["flightroute"];
                strlcpy(entry->origin_airport, route["origin"]["name"] | "", sizeof(entry->origin_airport));
                strlcpy(entry->destination_airport, route["destination"]["name"] | "", sizeof(entry->destination_airport));
                const char *airline = route["airline"]["name"] | "";
                if (airline[0]) strlcpy(entry->airline, airline, sizeof(entry->airline));
            }
        }
        http.end();
        http_mutex_release();
        notify_callback(entry);
    }

    // Stage 2: Aircraft details
    if (http_mutex_acquire(pdMS_TO_TICKS(8000))) {
        char url[128];
        snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/aircraft/%s", params->icao_hex);
        WiFiClientSecure client;
        client.setInsecure();
        client.setHandshakeTimeout(5);
        HTTPClient http;
        http.begin(client, url);
        http.setTimeout(5000);
        if (http.GET() == HTTP_CODE_OK) {
            String payload = http.getString();
            JsonDocument doc;
            if (!deserializeJson(doc, payload)) {
                JsonObject ac = doc["response"]["aircraft"];
                strlcpy(entry->manufacturer, ac["manufacturer"] | "", sizeof(entry->manufacturer));
                strlcpy(entry->model, ac["type"] | "", sizeof(entry->model));
                strlcpy(entry->owner, ac["registered_owner"] | "", sizeof(entry->owner));
                strlcpy(entry->registered_country, ac["registered_owner_country_name"] | "",
                        sizeof(entry->registered_country));
                entry->engine_count = ac["engine_count"] | 0;
                strlcpy(entry->engine_type, ac["engine_type"] | "", sizeof(entry->engine_type));
                entry->year_built = ac["year_built"] | 0;
            }
        }
        http.end();
        http_mutex_release();
        notify_callback(entry);
    }

    // Stage 3: Photo
    vTaskDelay(pdMS_TO_TICKS(200));
    if (http_mutex_acquire(pdMS_TO_TICKS(8000))) {
        char url[128];
        snprintf(url, sizeof(url),
                 "https://api.planespotters.net/pub/photos/hex/%s", params->icao_hex);
        WiFiClientSecure client;
        client.setInsecure();
        client.setHandshakeTimeout(5);
        HTTPClient http;
        http.begin(client, url);
        http.setTimeout(5000);
        if (http.GET() == HTTP_CODE_OK) {
            String payload = http.getString();
            JsonDocument doc;
            if (!deserializeJson(doc, payload)) {
                JsonArray photos = doc["photos"].as<JsonArray>();
                if (photos.size() > 0) {
                    strlcpy(entry->photo_url, photos[0]["thumbnail_large"]["src"] | "", sizeof(entry->photo_url));
                    strlcpy(entry->photo_photographer, photos[0]["photographer"] | "", sizeof(entry->photo_photographer));
                }
            }
        }
        http.end();
        http_mutex_release();
    }

    entry->loaded = true;
    entry->loading = false;
    notify_callback(entry);

    free(params);
    _task_running = false;
    vTaskDelete(nullptr);
}

void enrichment_fetch(const char *icao_hex, const char *registration,
                      const char *callsign,
                      void (*callback)(AircraftEnrichment *data)) {
    AircraftEnrichment *cached = enrichment_get_cached(icao_hex);
    if (cached) {
        callback(cached);
        return;
    }

    if (_task_running) return;

    _pending_callback = callback;
    _deferred_ready = false;

    EnrichParams *params = (EnrichParams *)malloc(sizeof(EnrichParams));
    strlcpy(params->icao_hex, icao_hex, sizeof(params->icao_hex));
    strlcpy(params->callsign, callsign ? callsign : "", sizeof(params->callsign));
    xTaskCreatePinnedToCore(fetch_task, "enrich", 8192, params, 0, nullptr, 1);
}

// Poll from main loop instead of LVGL timer
void enrichment_poll() {
    if (_deferred_ready && _pending_callback && _deferred_entry) {
        _deferred_ready = false;
        _pending_callback((AircraftEnrichment *)_deferred_entry);
    }
}

void enrichment_init() {
    // No-op on CYD (no LVGL timer — use enrichment_poll() from main loop)
}
