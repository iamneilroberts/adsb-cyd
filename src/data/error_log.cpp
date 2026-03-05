#include "error_log.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdarg.h>
#include <string.h>

static ErrorEntry _ring[ERROR_LOG_MAX];
static int _write_idx = 0;
static int _count = 0;         // entries in ring (max ERROR_LOG_MAX)
static uint32_t _total = 0;    // lifetime count
static SemaphoreHandle_t _mutex = nullptr;

void error_log_init() {
    _mutex = xSemaphoreCreateMutex();
    memset(_ring, 0, sizeof(_ring));
}

void error_log_add(const char *fmt, ...) {
    if (!_mutex) return;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;

    ErrorEntry &e = _ring[_write_idx];
    va_list args;
    va_start(args, fmt);
    vsnprintf(e.msg, ERROR_LOG_MSG_LEN, fmt, args);
    va_end(args);
    e.timestamp = millis();

    _write_idx = (_write_idx + 1) % ERROR_LOG_MAX;
    if (_count < ERROR_LOG_MAX) _count++;
    _total++;

    xSemaphoreGive(_mutex);
}

ErrorSnapshot error_log_snapshot() {
    ErrorSnapshot snap = {};
    if (!_mutex) return snap;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return snap;

    snap.count = _count;
    // Copy in chronological order (oldest first)
    int start = (_count < ERROR_LOG_MAX) ? 0 : _write_idx;
    for (int i = 0; i < _count; i++) {
        snap.entries[i] = _ring[(start + i) % ERROR_LOG_MAX];
    }

    xSemaphoreGive(_mutex);
    return snap;
}

void error_log_clear() {
    if (!_mutex) return;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    _count = 0;
    _write_idx = 0;
    memset(_ring, 0, sizeof(_ring));
    xSemaphoreGive(_mutex);
}

uint32_t error_log_total_count() {
    return _total;  // atomic read on ESP32
}
